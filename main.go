package main

import (
	"encoding/json"
	"fmt"
	"io"
	"log"
	"math"
	"net/http"
	"os"
	"strings"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/gildas/go-cache"
	"github.com/jessevdk/go-flags"
)

type Options struct {
	AircraftFile string  `short:"a" long:"aircraftfile" description:"Path to aircraft.json" default:"/run/dump1090-fa/aircraft.json" env:"AIRCRAFTFILE"`
	MqttBroker   string  `short:"b" long:"mqttbroker" description:"MQTT Broker URL" default:"tcp://localhost:1883" env:"MQTT_BROKER"`
	MqttTopic    string  `short:"c" long:"mqtttopic" description:"MQTT Topic" default:"homeassistant/sensor/aircraft" env:"MQTT_TOPIC"`
	MqttUsername string  `short:"u" long:"mqttusername" description:"MQTT username" env:"MQTT_USERNAME"`
	MqttPassword string  `short:"p" long:"mqttpassword" description:"MQTT password" env:"MQTT_PASSWORD"`
	HomeLat      float64 `short:"t" long:"homelat" description:"Home latitude" env:"LATITUDE"`
	HomeLon      float64 `short:"g" long:"homelon" description:"Home longitude" env:"LONGITUDE"`
}

var cliOptions Options
var parser = flags.NewParser(&cliOptions, flags.Default)

type Aircraft struct {
	Hex              string
	Flight           string
	Lat              float64
	Lon              float64
	Track            float64
	Alt_Baro         int64
	Direction        string
	Mach             float64
	Speed            float64
	Squawk           string
	Emergency        string
	Manufacturer     string // from https://hexdb.io/api/v1/aircraft
	Type             string // from https://hexdb.io/api/v1/aircraft
	RegisteredOwners string // from https://hexdb.io/api/v1/aircraft
	Route            string // from https://hexdb.io/api/v1/route/icao
	From             string // from Route + https://hexdb.io/api/v1/airport/icao
	To               string // from Route + https://hexdb.io/api/v1/airport/icao
	Distance         float64
}
type Dump struct {
	Now      float64
	Messages int64
	Aircraft []Aircraft
}

type AircraftCache struct {
	Manufacturer     string
	Type             string
	RegisteredOwners string
}

var aircraftCache *cache.Cache[AircraftCache]

type RouteCache struct {
	Route string
}

var routeCache *cache.Cache[RouteCache]

type AirportCache struct {
	Region_Name string
	Airport     string
}

var airportCache *cache.Cache[AirportCache]

var connectHandler mqtt.OnConnectHandler = func(client mqtt.Client) {
	log.Println("Connected to MQTT Broker")
}

var connectLostHandler mqtt.ConnectionLostHandler = func(client mqtt.Client, err error) {
	log.Printf("Connection lost: %v", err)
}

var Directions = [17]string{"N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE", "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW", "N"}

func main() {

	// Parse flags
	//
	_, err := parser.Parse()
	if err != nil {
		panic(fmt.Sprintf("could not parse cli: %v", err))
	}

	// caches
	//
	aircraftCache = cache.New[AircraftCache]("aircraft", cache.CacheOptionPersistent).WithExpiration(7 * 24 * time.Hour)
	routeCache = cache.New[RouteCache]("route", cache.CacheOptionPersistent).WithExpiration(7 * 24 * time.Hour)
	airportCache = cache.New[AirportCache]("airport", cache.CacheOptionPersistent).WithExpiration(7 * 24 * time.Hour)

	// MQTT
	//
	opts := mqtt.NewClientOptions()
	opts.AddBroker(cliOptions.MqttBroker)
	opts.SetClientID("dump1090")
	opts.SetUsername(cliOptions.MqttUsername)
	opts.SetPassword(cliOptions.MqttPassword)
	opts.OnConnect = connectHandler
	opts.OnConnectionLost = connectLostHandler
	client := mqtt.NewClient(opts)
	if token := client.Connect(); token.Wait() && token.Error() != nil {
		panic(token.Error())
	}

	var oldAircraft = make(map[string]int)

	var lastNow = 0.0

	// loop
	for {
		var newAircraft = make(map[string]int)

		time.Sleep(time.Second * 1)

		// Read dump file
		//
		var dump Dump
		data, err := os.ReadFile(cliOptions.AircraftFile)
		if err != nil {
			log.Fatal(err)
		}
		err = json.Unmarshal(data, &dump)
		if err != nil {
			log.Fatal(err)
		}
		log.Printf("Now: %f Messages: %d\n", dump.Now, dump.Messages)

		if dump.Now <= lastNow {
			continue
		}
		lastNow = dump.Now

		for _, aircraft := range dump.Aircraft {

			aircraft.Flight = strings.ReplaceAll(aircraft.Flight, " ", "")

			// aircraft
			//
			hexdbAircraft := getAircraft(aircraft.Hex)
			aircraft.Manufacturer = hexdbAircraft.Manufacturer
			aircraft.Type = hexdbAircraft.Type
			aircraft.RegisteredOwners = hexdbAircraft.RegisteredOwners

			// route
			//
			if len(aircraft.Flight) > 0 {
				hexdbRoute := getRoute(aircraft.Flight)
				aircraft.Route = hexdbRoute.Route
			}

			// airport
			//
			airports := strings.Split(aircraft.Route, "-")
			if len(airports) == 2 {
				hexdbAirport := getAirport(airports[0])
				aircraft.From = hexdbAirport.Airport + "," + hexdbAirport.Region_Name
				hexdbAirport = getAirport(airports[1])
				aircraft.To = hexdbAirport.Airport + "," + hexdbAirport.Region_Name
			}

			// https://hexdb.io/hex-image?hex=4cadb9 - image and https://hexdb.io/hex-image-thumb?hex=4cadb9
			// is this useful ??

			// add distance from base (miles)
			//
			if aircraft.Lat != 0.0 && aircraft.Lon != 0.0 {
				aircraft.Distance = Distance(aircraft.Lat, aircraft.Lon, cliOptions.HomeLat, cliOptions.HomeLon) / 1609.344
			}

			// track -> text
			//
			aircraft.Direction = Directions[int((aircraft.Track+11.25)/22.5)]

			// mach -> speed
			//
			if aircraft.Mach > 0 {
				aircraft.Speed = aircraft.Mach * 767.269148
			}

			log.Printf("Hex: %s Flight: %s Lat: %f Long: %f Distance: %f Track: %f Mach: %f Aircraft: %s %s Owner: %s Route: %s (%s -> %s)\n", aircraft.Hex, aircraft.Flight, aircraft.Lat, aircraft.Lon, aircraft.Distance, aircraft.Track, aircraft.Mach, aircraft.Manufacturer, aircraft.Type, aircraft.RegisteredOwners, aircraft.Route, aircraft.From, aircraft.To)

			// Push to mqtt - keyed by hex ?
			//
			token := client.Publish(cliOptions.MqttTopic+"/"+aircraft.Hex+"/config", 0, false, "{ \"name\": \"aircraft-"+aircraft.Hex+"\", \"icon\": \"mdi:aircraft\", \"state_topic\": \""+cliOptions.MqttTopic+"/"+aircraft.Hex+"/state\", \"json_attributes_topic\": \""+cliOptions.MqttTopic+"/"+aircraft.Hex+"/attributes\"}")
			token.Wait()
			b, err := json.Marshal(aircraft)
			if err != nil {
				fmt.Println(err)
				return
			}
			token = client.Publish(cliOptions.MqttTopic+"/"+aircraft.Hex+"/state", 0, false, "ok")
			token.Wait()
			token = client.Publish(cliOptions.MqttTopic+"/"+aircraft.Hex+"/attributes", 0, false, string(b))
			token.Wait()

			newAircraft[aircraft.Hex] = 1
		}

		// Remove any old flights
		//
		for hex := range oldAircraft {

			_, ok := newAircraft[hex]
			if !ok {
				// found in oldAircraft but not in aircraftCache
				log.Printf("Need to delete %s since it seems to have disapeared\n", hex)
				delete(oldAircraft, hex)
				delete(newAircraft, hex)
				token := client.Publish(cliOptions.MqttTopic+"/"+hex+"/state", 0, false, "")
				token.Wait()
				token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/attributes", 0, false, "")
				token.Wait()
				token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/config", 0, false, "")
				token.Wait()
			}
		}
		for _, aircraft := range dump.Aircraft {
			oldAircraft[aircraft.Hex] = 1
		}

	}

	// delete all on quit
	//
	/*
		for _, aircraft := range dump.Aircraft {
			token := client.Publish(cliOptions.MqttTopic+"/"+aircraft.Hex+"/state", 0, false, "")
			token.Wait()
			token = client.Publish(cliOptions.MqttTopic+"/"+aircraft.Hex+"/attributes", 0, false, "")
			token.Wait()
			token = client.Publish(cliOptions.MqttTopic+"/"+aircraft.Hex+"/config", 0, false, "")
			token.Wait()
		}
	*/

}

// Get aircraft info
//
// https://hexdb.io/api/v1/aircraft/4cadb9 - Manufacturer / Type / Owner
func getAircraft(hex string) *AircraftCache {
	httpClient := http.Client{
		Timeout: time.Second * 2, // Timeout after 2 seconds
	}

	hexdbAircraft, err := aircraftCache.Get(hex)
	if err != nil {
		req, err := http.NewRequest(http.MethodGet, "https://hexdb.io/api/v1/aircraft/"+hex, nil)
		if err != nil {
			log.Fatal(err)
		}
		res, getErr := httpClient.Do(req)
		if getErr != nil {
			log.Fatal(getErr)
		}
		if res.Body != nil {
			defer res.Body.Close()
		}
		body, readErr := io.ReadAll(res.Body)
		if readErr != nil {
			log.Fatal(readErr)
		}
		jsonErr := json.Unmarshal(body, &hexdbAircraft)
		if jsonErr != nil {
			log.Fatal(jsonErr)
		}
		aircraftCache.Set(*hexdbAircraft, hex)
	}

	return hexdbAircraft
}

// Get route info
//
// https://hexdb.io/api/v1/route/icao/RYR3EV - route
func getRoute(flight string) *RouteCache {
	httpClient := http.Client{
		Timeout: time.Second * 2, // Timeout after 2 seconds
	}

	hexdbRoute, err := routeCache.Get(flight)
	if err != nil {
		req, err := http.NewRequest(http.MethodGet, "https://hexdb.io/api/v1/route/icao/"+flight, nil)
		if err != nil {
			log.Fatal(err)
		}
		res, getErr := httpClient.Do(req)
		if getErr != nil {
			log.Fatal(getErr)
		}
		if res.Body != nil {
			defer res.Body.Close()
		}
		body, readErr := io.ReadAll(res.Body)
		if readErr != nil {
			log.Fatal(readErr)
		}
		jsonErr := json.Unmarshal(body, &hexdbRoute)
		if jsonErr != nil {
			log.Fatal(jsonErr)
		}
		routeCache.Set(*hexdbRoute, flight)
	}

	return hexdbRoute
}

// Get airport info
//
// https://hexdb.io/api/v1/airport/icao/EGSS - airport / region
func getAirport(icao string) *AirportCache {
	httpClient := http.Client{
		Timeout: time.Second * 2, // Timeout after 2 seconds
	}

	hexdbAirport, err := airportCache.Get(icao)
	if err != nil {
		req, err := http.NewRequest(http.MethodGet, "https://hexdb.io/api/v1/airport/icao/"+icao, nil)
		if err != nil {
			log.Fatal(err)
		}
		res, getErr := httpClient.Do(req)
		if getErr != nil {
			log.Fatal(getErr)
		}
		if res.Body != nil {
			defer res.Body.Close()
		}
		body, readErr := io.ReadAll(res.Body)
		if readErr != nil {
			log.Fatal(readErr)
		}
		jsonErr := json.Unmarshal(body, &hexdbAirport)
		if jsonErr != nil {
			log.Fatal(jsonErr)
		}
		airportCache.Set(*hexdbAirport, icao)
	}

	return hexdbAirport
}

// haversin(θ) function
func hsin(theta float64) float64 {
	return math.Pow(math.Sin(theta/2), 2)
}

// Distance function returns the distance (in meters) between two points of
//
//	a given longitude and latitude relatively accurately (using a spherical
//	approximation of the Earth) through the Haversin Distance Formula for
//	great arc distance on a sphere with accuracy for small distances
//
// point coordinates are supplied in degrees and converted into rad. in the func
//
// distance returned is METERS!!!!!!
// http://en.wikipedia.org/wiki/Haversine_formula
func Distance(lat1, lon1, lat2, lon2 float64) float64 {
	// convert to radians
	// must cast radius as float to multiply later
	var la1, lo1, la2, lo2, r float64
	la1 = lat1 * math.Pi / 180
	lo1 = lon1 * math.Pi / 180
	la2 = lat2 * math.Pi / 180
	lo2 = lon2 * math.Pi / 180

	r = 6378100 // Earth radius in METERS

	// calculate
	h := hsin(la2-la1) + math.Cos(la1)*math.Cos(la2)*hsin(lo2-lo1)

	return 2 * r * math.Asin(math.Sqrt(h))
}
