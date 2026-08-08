package main

import (
	"encoding/json"
	"fmt"
	"io"
	"log"
	"math"
	"net/http"
	"os"
	"os/signal"
	"strings"
	"syscall"
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

/*
hex: the 24-bit ICAO identifier of the aircraft, as 6 hex digits. The identifier may start with '~', this means that the address is a non-ICAO address (e.g. from TIS-B).
type: type of underlying messages / best source of current data for this position / aircraft: (the following list is in order of which data is preferentially used)
adsb_icao: messages from a Mode S or ADS-B transponder, using a 24-bit ICAO address
	adsb_icao_nt: messages from an ADS-B equipped "non-transponder" emitter e.g. a ground vehicle, using a 24-bit ICAO address
	adsr_icao: rebroadcast of ADS-B messages originally sent via another data link e.g. UAT, using a 24-bit ICAO address
	tisb_icao: traffic information about a non-ADS-B target identified by a 24-bit ICAO address, e.g. a Mode S target tracked by secondary radar
	adsc: ADS-C (received by monitoring satellite downlinks)
	mlat: MLAT, position calculated arrival time differences using multiple receivers, outliers and varying accuracy is expected.
	other: miscellaneous data received via Basestation / SBS format, quality / source is unknown.
	mode_s: ModeS data from the planes transponder (no position transmitted)
	adsb_other: messages from an ADS-B transponder using a non-ICAO address, e.g. anonymized address
	adsr_other: rebroadcast of ADS-B messages originally sent via another data link e.g. UAT, using a non-ICAO address
	tisb_other: traffic information about a non-ADS-B target using a non-ICAO address
	tisb_trackfile: traffic information about a non-ADS-B target using a track/file identifier, typically from primary or Mode A/C radar
flight: callsign, the flight name or aircraft registration as 8 chars (2.2.8.2.6)
alt_baro: the aircraft barometric altitude in feet as a number OR "ground" as a string
alt_geom: geometric (GNSS / INS) altitude in feet referenced to the WGS84 ellipsoid
gs: ground speed in knots
ias: indicated air speed in knots
tas: true air speed in knots
mach: Mach number
track: true track over ground in degrees (0-359)
track_rate: Rate of change of track, degrees/second
roll: Roll, degrees, negative is left roll
mag_heading: Heading, degrees clockwise from magnetic north
true_heading: Heading, degrees clockwise from true north (usually only transmitted on ground, in the air usually derived from the magnetic heading using magnetic model WMM2020)
baro_rate: Rate of change of barometric altitude, feet/minute
geom_rate: Rate of change of geometric (GNSS / INS) altitude, feet/minute
squawk: Mode A code (Squawk), encoded as 4 octal digits
	https://freedar.uk/squawk-codes/
emergency: ADS-B emergency/priority status, a superset of the 7x00 squawks (2.2.3.2.7.8.1.1) (none, general, lifeguard, minfuel, nordo, unlawful, downed, reserved)
category: emitter category to identify particular aircraft or vehicle classes (values A0 - D7) (2.2.3.2.5.2)
	A0 : No ADS-B emitter category information. Do not use this emitter category. If no emitter category fits your installation, seek guidance from the FAA as appropriate.
	A1 : Light (< 15500 lbs) – Any airplane with a maximum takeoff weight less than 15,500 pounds. This includes very light aircraft (light sport aircraft) that do not meet the requirements of 14 CFR § 103.1.
	A2 : Small (15500 to 75000 lbs) – Any airplane with a maximum takeoff weight greater than or equal to15,500 pounds but less than 75,000 pounds.
	A3 : Large (75000 to 300000 lbs) – Any airplane with a maximum takeoff weight greater than or equal to 75,000 pounds but less than 300,000 pounds that does not qualify for the high vortex category.
	A4 : High vortex large (aircraft such as B-757) – Any airplane with a maximum takeoff weight greater than or equal to 75,000 pounds but less than 300,000 pounds that has been determined to generate a high wake vortex. Currently, the Boeing 757 is the only example.
	A5 : Heavy (> 300000 lbs) – Any airplane with a maximum takeoff weight equal to or above 300,000 pounds.
	A6 : High performance (> 5g acceleration and 400 kts) – Any airplane, regardless of weight, which can maneuver in excess of 5 G’s and maintain true airspeed above 400 knots.
	A7 : Rotorcraft – Any rotorcraft regardless of weight.
	B0 : No ADS-B emitter category information
	B1 : Glider / sailplane – Any glider or sailplane regardless of weight.
	B2 : Lighter-than-air – Any lighter than air (airship or balloon) regardless of weight.
	B3 : Parachutist / skydiver
	B4 : Ultralight / hang-glider / paraglider – A vehicle that meets the requirements of 14 CFR § 103.1. Light sport aircraft should not use the ultralight emitter category unless they meet 14 CFR § 103.1.
	B5 : Reserved
	B6 : Unmanned aerial vehicle – Any unmanned aerial vehicle or unmanned aircraft system regardless of weight.
	B7 : Space / trans-atmospheric vehicle
	C0 : No ADS-B emitter category information
	C1 : Surface vehicle – emergency vehicle
	C2 : Surface vehicle – service vehicle
	C3 : Point obstacle (includes tethered balloons)
	C4 : Cluster obstacle
	C5 : Line obstacle
	C6 : Reserved
	C7 : Reserved
nav_qnh: altimeter setting (QFE or QNH/QNE), hPa
nav_altitude_mcp: selected altitude from the Mode Control Panel / Flight Control Unit (MCP/FCU) or equivalent equipment
nav_altitude_fms: selected altitude from the Flight Manaagement System (FMS) (2.2.3.2.7.1.3.3)
nav_heading: selected heading (True or Magnetic is not defined in DO-260B, mostly Magnetic as that is the de facto standard) (2.2.3.2.7.1.3.7)
nav_modes: set of engaged automation modes: 'autopilot', 'vnav', 'althold', 'approach', 'lnav', 'tcas'
lat, lon: the aircraft position in decimal degrees
nic: Navigation Integrity Category (2.2.3.2.7.2.6)
rc: Radius of Containment, meters; a measure of position integrity derived from NIC & supplementary bits. (2.2.3.2.7.2.6, Table 2-69)
seen_pos: how long ago (in seconds before "now") the position was last updated
track: true track over ground in degrees (0-359)
version: ADS-B Version Number 0, 1, 2 (3-7 are reserved) (2.2.3.2.7.5)
nic_baro: Navigation Integrity Category for Barometric Altitude (2.2.5.1.35)
nac_p: Navigation Accuracy for Position (2.2.5.1.35)
nac_v: Navigation Accuracy for Velocity (2.2.5.1.19)
sil: Source Integity Level (2.2.5.1.40)
sil_type: interpretation of SIL: unknown, perhour, persample
gva: Geometric Vertical Accuracy (2.2.3.2.7.2.8)
sda: System Design Assurance (2.2.3.2.7.2.4.6)
mlat: list of fields derived from MLAT data
tisb: list of fields derived from TIS-B data
messages: total number of Mode S messages received from this aircraft
seen: how long ago (in seconds before "now") a message was last received from this aircraft
rssi: recent average RSSI (signal power), in dbFS; this will always be negative.
alert: Flight status alert bit (2.2.3.2.3.2)
spi: Flight status special position identification bit (2.2.3.2.3.2)
wd, ws: wind direction and wind speed are calculated from ground track, true heading, true airspeed and ground speed
oat, tat: outer/static air temperature (C) and total air temperature (C) are calculated from mach number and true airspeed (typically somewhat inaccurate at lower altitudes / mach numbers below 0.5, calculation is inhibited for mach < 0.395)
acas_ra: experimental, subject to change, see format here:
	readsb/json_out.c

	Line 249 in ca5b825

 	static char *sprintACASJson(char *p, char *end, unsigned char *bytes, struct modesMessage *mm, int64_t now) {
gpsOkBefore: experimental, subject to change: aircraft lost GPS / GPS heavily degraded, it was working well before this timestamp, only displayed for 15 min after GPS is lost / degraded

*/

type Aircraft struct {
	Hex              string
	Flight           string
	Lat              float64
	Lon              float64
	Track            float64
	Alt_Baro         int64
	Direction        string
	Gs               float64
	Speed            float64
	Squawk           string
	Emergency        string
	Category         string
	Manufacturer     string
	Type             string
	RegisteredOwners string
	From             string
	To               string
	Distance         float64
}
type Dump struct {
	Now      float64
	Messages int64
	Aircraft []Aircraft
}

type AircraftB struct {
	Manufacturer     string
	Type             string
	Registered_Owner string
}
type AircraftA struct {
	Aircraft AircraftB
}
type AircraftCache struct {
	Response AircraftA
}

var aircraftCache *cache.Cache[AircraftCache]

type Airport struct {
	Country_Name string
	Municipality string
	Name         string
}

type Airline struct {
	Name string
}
type RouteB struct {
	Airline     Airline
	Origin      Airport
	Destination Airport
}
type RouteA struct {
	Flightroute RouteB
}
type RouteCache struct {
	Response RouteA
}

var routeCache *cache.Cache[RouteCache]

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

	// alert topic
	//
	token := client.Publish(cliOptions.MqttTopic+"/alert/config", 0, false, "{ \"name\": \"aircraft-alert\", \"icon\": \"mdi:airplane\", \"state_topic\": \""+cliOptions.MqttTopic+"/alert/state\", \"json_attributes_topic\": \""+cliOptions.MqttTopic+"/alert/attributes\", \"unique_id\": \"aircraft-alert\"}")
	token.Wait()
	token = client.Publish(cliOptions.MqttTopic+"/alert/state", 0, false, "{}")
	token.Wait()

	var oldAircraft = make(map[string]int)
	var alertedAircraft = make(map[string]int)

	// cleanup on quit
	//
	c := make(chan os.Signal, 1)
	signal.Notify(c, os.Interrupt, syscall.SIGTERM, syscall.SIGABRT, syscall.SIGQUIT, syscall.SIGPIPE, syscall.SIGCHLD, syscall.SIGHUP, syscall.SIGINT)
	go func() {
		<-c
		cleanup(client, oldAircraft)
		os.Exit(1)
	}()

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
			log.Println(err)
			continue
		}
		err = json.Unmarshal(data, &dump)
		if err != nil {
			log.Println(err)
			continue
		}
		log.Printf("Now: %f Messages: %d\n", dump.Now, dump.Messages)

		if dump.Now <= lastNow {
			continue
		}
		lastNow = dump.Now

		for _, aircraft := range dump.Aircraft {

			aircraft.Flight = strings.ReplaceAll(aircraft.Flight, " ", "")

			// add distance from base (miles)
			//
			distance := -1.0
			if aircraft.Lat != 0.0 && aircraft.Lon != 0.0 {
				aircraft.Distance = Distance(aircraft.Lat, aircraft.Lon, cliOptions.HomeLat, cliOptions.HomeLon) / 1609.344
				distance = aircraft.Distance
			}

			// track -> text
			//
			aircraft.Direction = Directions[int((aircraft.Track+11.25)/22.5)]

			// gs -> speed
			//
			if aircraft.Gs > 0 {
				aircraft.Speed = aircraft.Gs * 1.150779
			}

			// FIX THIS - expand category
			//

			// FIX THIS - watch out for sending multiple requests for data that doesn't exist
			// aircraft
			//
			adsbdbAircraft := getAircraft(aircraft.Hex, distance > 0 && distance < 10)
			if adsbdbAircraft != nil {
				aircraft.Manufacturer = adsbdbAircraft.Response.Aircraft.Manufacturer
				aircraft.Type = adsbdbAircraft.Response.Aircraft.Type
				aircraft.RegisteredOwners = adsbdbAircraft.Response.Aircraft.Registered_Owner
			}

			// route
			//
			if len(aircraft.Flight) > 0 {
				adsbdbRoute := getRoute(aircraft.Flight, distance > 0 && distance < 10)
				if adsbdbRoute != nil {
					aircraft.From = adsbdbRoute.Response.Flightroute.Origin.Name + "," + adsbdbRoute.Response.Flightroute.Origin.Country_Name
					aircraft.To = adsbdbRoute.Response.Flightroute.Destination.Name + "," + adsbdbRoute.Response.Flightroute.Destination.Country_Name
				}
			}

			log.Printf("Hex: %s Flight: %s Lat: %f Long: %f Distance: %f Track: %f Gs: %f Aircraft: %s %s Owner: %s Route: %s -> %s\n", aircraft.Hex, aircraft.Flight, aircraft.Lat, aircraft.Lon, aircraft.Distance, aircraft.Track, aircraft.Gs, aircraft.Manufacturer, aircraft.Type, aircraft.RegisteredOwners, aircraft.From, aircraft.To)

			// Push to mqtt
			//
			token := client.Publish(cliOptions.MqttTopic+"/"+aircraft.Hex+"/config", 0, false, "{ \"name\": \"aircraft-"+aircraft.Hex+"\", \"icon\": \"mdi:airplane\", \"state_topic\": \""+cliOptions.MqttTopic+"/"+aircraft.Hex+"/state\", \"json_attributes_topic\": \""+cliOptions.MqttTopic+"/"+aircraft.Hex+"/attributes\", \"unique_id\": \"aircraft-"+aircraft.Hex+"\"}")
			token.Wait()
			b, err := json.Marshal(aircraft)
			if err != nil {
				fmt.Println(err)
				return
			}

			// alert if aircraft is close but not already sent an alert
			_, alerted := alertedAircraft[aircraft.Hex]
			if distance > 0 && distance < 5 && !alerted {
				token = client.Publish(cliOptions.MqttTopic+"/alert/attributes", 0, false, string(b))
				token.Wait()
				alertedAircraft[aircraft.Hex] = 1
			}
			if distance >= 5 {
				delete(alertedAircraft, aircraft.Hex)
			}

			token = client.Publish(cliOptions.MqttTopic+"/"+aircraft.Hex+"/state", 0, false, "{}")
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
				log.Printf("Deleting %s since it seems to have disapeared\n", hex)
				delete(oldAircraft, hex)
				delete(newAircraft, hex)
				token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/config", 0, false, "{}")
				token.Wait()
				token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/config", 0, false, "")
				token.Wait()
				token := client.Publish(cliOptions.MqttTopic+"/"+hex+"/state", 0, false, "")
				token.Wait()
				token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/attributes", 0, false, "{}")
				token.Wait()
				token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/attributes", 0, false, "")
				token.Wait()
			}
		}
		for _, aircraft := range dump.Aircraft {
			oldAircraft[aircraft.Hex] = 1
		}

	}

}

func cleanup(client mqtt.Client, oldAircraft map[string]int) {
	// delete all on quit
	//
	log.Println("Cleanup")
	for hex := range oldAircraft {
		log.Println("Removing " + hex)

		token := client.Publish(cliOptions.MqttTopic+"/"+hex+"/config", 0, false, "{}")
		token.Wait()
		token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/config", 0, false, "")
		token.Wait()
		token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/state", 0, false, "")
		token.Wait()
		token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/attributes", 0, false, "{}")
		token.Wait()
		token = client.Publish(cliOptions.MqttTopic+"/"+hex+"/attributes", 0, false, "")
		token.Wait()
	}
}

// Get aircraft info
func getAircraft(hex string, fetch bool) *AircraftCache {
	httpClient := http.Client{
		Timeout: time.Second * 2, // Timeout after 2 seconds
	}

	adsbdbAircraft, err := aircraftCache.Get(hex)
	if fetch && err != nil {
		req, err := http.NewRequest(http.MethodGet, "https://api.adsbdb.com/v0/aircraft/"+hex, nil)
		if err != nil {
			log.Println(err)
			return nil
		}
		res, err := httpClient.Do(req)
		if err != nil {
			log.Println(err)
			return nil
		}
		if res.Body != nil {
			defer res.Body.Close()
		}
		body, err := io.ReadAll(res.Body)
		if err != nil {
			log.Println(err)
			return nil
		}
		err = json.Unmarshal(body, &adsbdbAircraft)
		if err != nil {
			log.Println(err)
			return nil
		}
		aircraftCache.Set(*adsbdbAircraft, hex)
	}

	return adsbdbAircraft
}

// Get route info
func getRoute(flight string, fetch bool) *RouteCache {
	httpClient := http.Client{
		Timeout: time.Second * 2, // Timeout after 2 seconds
	}

	adsbdbRoute, err := routeCache.Get(flight)
	if fetch && err != nil {
		req, err := http.NewRequest(http.MethodGet, "https://api.adsbdb.com/v0/callsign/"+flight, nil)
		if err != nil {
			log.Println(err)
			return nil
		}
		res, err := httpClient.Do(req)
		if err != nil {
			log.Println(err)
			return nil
		}
		if res.Body != nil {
			defer res.Body.Close()
		}
		body, err := io.ReadAll(res.Body)
		if err != nil {
			log.Println(err)
			return nil
		}
		err = json.Unmarshal(body, &adsbdbRoute)
		if err != nil {
			log.Println(err)
			return nil
		}
		routeCache.Set(*adsbdbRoute, flight)
	}

	return adsbdbRoute
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
