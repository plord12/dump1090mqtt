# environment file containing test data
#
-include env
export

NAME=dump1090mqtt
BINDIR=bin
SOURCES=$(wildcard *.go)
BINARIES=${BINDIR}/${NAME} ${BINDIR}/${NAME}-linux-arm64

all: ${BINDIR} ${BINARIES}

${BINDIR}:
	mkdir -p ${BINDIR}
	
${BINDIR}/${NAME}: ${SOURCES}
	go build -o $@ $^

${BINDIR}/${NAME}-darwin-amd64: ${SOURCES}
	GOARCH=amd64 GOOS=darwin go build -o $@ $^

${BINDIR}/${NAME}-darwin-arm64: ${SOURCES}
	GOARCH=arm64 GOOS=darwin go build -o $@ $^

${BINDIR}/${NAME}-darwin: ${BINDIR}/${NAME}-darwin-amd64 ${BINDIR}/${NAME}-darwin-arm64
	makefat $@ $^

${BINDIR}/${NAME}-linux-amd64: ${SOURCES}
	GOARCH=amd64 GOOS=linux go build -o $@ $^

${BINDIR}/${NAME}-linux-arm64: ${SOURCES}
	GOARCH=arm64 GOOS=linux go build -o $@ $^

${BINDIR}/${NAME}-linux-arm: ${SOURCES}
	GOARCH=arm GOOS=linux go build -o $@ $^

${BINDIR}/${NAME}-windows.exe: ${SOURCES}
	GOARCH=amd64 GOOS=windows go build -o $@ $^

test: ${BINDIR}/${NAME}
	${BINDIR}/${NAME} -a aircraft.json

clean:
	@go clean
	-@rm -rf ${BINDIR} 2>/dev/null || true
