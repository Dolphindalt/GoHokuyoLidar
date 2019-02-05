package gohokuyolidar

import (
	"bytes"
	"errors"
	"log"
	"strconv"

	serial "github.com/mikepb/go-serial"
)

// lsusb
// sudo chmod a+rw /dev/ttyACM0
const (
	lf            byte = 0x0a
	cr            byte = 0x0d
	version       byte = 0x56
	segmentSize   int  = 65
	measureTag    byte = 0x4d
	threeEncoding byte = 0x44
	twoEncoding   byte = 0x53
)

var healthStatus = map[string]string{
	"00": "Command received without any Error",
	"01": "Starting Step has non-numeric value",
	"02": "End Step has non-numeric value",
	"03": "Cluster Count has non-numeric value",
	"04": "End Step is out of range",
	"05": "End Step is smaller than Starting Step",
	"06": "Scan Interval has non-numeric value",
	"07": "Number of Scan has non-numeric value",
	"98": "Resumption of process after confirming normal laser operation",
}

// HokuyoLidar represents the lidar structure
type HokuyoLidar struct {
	// lidar related data
	serialPort  *serial.Port
	portName    string
	baudrate    int
	MotorActive bool
	options     *serial.Options
	Connected   bool
	Scanning    bool

	// scan operation related data
	startStep    int
	endStep      int
	clusterCount int
	scanInterval int
	encodingType byte
	headSize     int
	requestTag   byte
}

// NewHokuyoLidar creates an instance of the lidar struct
func NewHokuyoLidar(portName string, baudrate int) *HokuyoLidar {
	return &HokuyoLidar{nil, portName, baudrate, false, nil, false, false,
		0, 0, 0, 0, 0, 0, 0}
}

// Connect activates the serial port connection to the lidar
// Some devices run scip 1.1 by default. If so, specify scip1IsDefault as true
func (h *HokuyoLidar) Connect(scip1IsDefault bool) error {
	if h.Connected {
		err := errors.New("Lidar is already connected")
		return err
	}
	options := serial.RawOptions
	options.Mode = serial.MODE_READ_WRITE
	options.BitRate = h.baudrate

	serialPort, err := options.Open(h.portName)
	if err != nil {
		return err
	}
	h.options = &options
	h.serialPort = serialPort
	h.Connected = true

	if scip1IsDefault {
		h.scipTwoCmd()
	}

	return nil
}

// Disconnect disables the serial port connection to the lidar
func (h *HokuyoLidar) Disconnect() error {
	if h.Connected {
		return errors.New("Lidar is already connected")
	}
	h.serialPort.Reset()
	err := h.serialPort.Close()
	if err != nil {
		return err
	}
	h.Connected = false
	return nil
}

// S C I P 2 . 0 LF
// S C I P 2 . 0 LF STATUS LF LF
func (h *HokuyoLidar) scipTwoCmd() {
	cmd := []byte{'S', 'C', 'I', 'P', '2', '.', '0', lf}
	err := h.sendCommandBlock(cmd)
	if err != nil {
		log.Fatalf("Failed to init scip 2.0 protocol: %v\n", err)
	}
	_, res, err := h.readFixedResponse(13)
	if err != nil {
		log.Fatalf("ScipTwoCmd: %v\n", err)
	}
	var buffer bytes.Buffer
	buffer.Write(res[9:10])
	statusCode := buffer.String()
	log.Printf("%v\n", res)
	statusCheck(statusCode)
}

// VersionCmd asks the lidar for it's version info
func (h *HokuyoLidar) VersionCmd() (string, error) {
	cmd := []byte{version, lf}
	h.sendCommandBlock(cmd)
	read, bytes, err := h.readFixedResponse(7 * segmentSize)
	if err != nil {
		return "", err
	}
	msg := string(bytes[:read])
	return msg, nil
}

// MDMSCmd is a sensor data aquisition command that uses three character encoding
func (h *HokuyoLidar) MDMSCmd(three bool, startStep, endStep, clusterCount, scanInterval, numberOfScans int, characters string) {
	// stupid proofing the scan
	ss := strconv.Itoa(startStep)
	es := strconv.Itoa(endStep)
	cc := strconv.Itoa(clusterCount)
	si := strconv.Itoa(scanInterval)
	ns := strconv.Itoa(numberOfScans)

	zeroPadString(4, &ss)
	zeroPadString(4, &es)
	zeroPadString(2, &cc)
	zeroPadString(1, &si)
	zeroPadString(2, &ns)
	log.Printf("Scans: %v\n", ns)
	if len(characters) > 16 {
		characters = characters[0:16]
	}

	var encode byte
	if three {
		encode = threeEncoding
	} else {
		encode = twoEncoding
	}

	cmd := []byte{measureTag, encode}
	cmd = append(cmd[:], []byte(ss)[:]...)
	cmd = append(cmd[:], []byte(es)[:]...)
	cmd = append(cmd[:], []byte(cc)[:]...)
	cmd = append(cmd[:], []byte(si)[:]...)
	cmd = append(cmd[:], []byte(ns)[:]...)
	cmd = append(cmd[:], []byte(characters)[:]...)
	cmd = append(cmd[:], lf)

	log.Printf("Cmd: %v\n", cmd)
	err := h.sendCommandBlock(cmd)
	if err != nil {
		log.Fatalf("Encountered error during MD init: %v\n", err)
	}
	headLen := 21 + len(characters)
	_, head, err := h.readFixedResponse(headLen)
	if err != nil {
		log.Fatalf("Err in scan init: %v\n", err)
	}
	log.Printf("Data: %v\n", head)
	statusCode := head[headLen-5 : headLen-3]
	log.Printf("Status: %v\n", statusCode)
	statusCheck(string(statusCode))

	h.startStep = startStep
	h.endStep = endStep
	h.clusterCount = clusterCount
	h.scanInterval = scanInterval
	h.encodingType = threeEncoding
	h.headSize = 26
	h.requestTag = measureTag
}

// GetDistanceAndIntensity returns a list of distances, intensities, and a timestamp
func (h *HokuyoLidar) GetDistanceAndIntensity() ([]int, []int, int) {
	log.Printf("t1\n")
	resLen := int(h.headSize - 10)
	_, _, err := h.readFixedResponse(resLen)
	if err != nil {
		log.Fatalf("Failed to read reponse header: %v\n", err)
	}
	log.Printf("t2\n")
	_, statusAndJunk, err := h.readFixedResponse(4)
	if err != nil {
		log.Fatalf("Failed to read status of scan: %v\n", err)
	}
	log.Printf("t3\n")
	statusCode := string(statusAndJunk[0:2])
	statusCheck(statusCode)
	log.Printf("t4\n")

	_, encodedTime, err := h.readFixedResponse(6)
	log.Printf("t5\n")
	if err != nil {
		log.Fatalf("Failed to read timestamp: %v\n", err)
	}
	timestamp := decode(encodedTime)

	pointDx := h.endStep - h.startStep + 1

	data := []byte{}
	for {
		_, chungus, err := h.readFixedResponse(66) // data plus lf lf
		if err != nil {
			log.Fatalf("Failed to read data chunk during scan: %v\n", err)
		}
		data = append(data[:], chungus[0:len(chungus)-2]...)
		dataleft := string(chungus[13:15])
		log.Printf("Chungus %v\n", chungus)
		log.Printf("Data Left %v\n", dataleft)
		if dataleft == "00" {
			break
		}
	}

	oddball := pointDx * 6 % 64
	if oddball != 0 { // read remaining data chunk
		log.Printf("t8\n")
		_, chungus, err := h.readFixedResponse(oddball + 2)
		if err != nil {
			log.Fatalf("Failed to read straggling data chunk during scan: %v\n", err)
		}
		data = append(data[:], chungus[0:len(chungus)-2]...)
	}

	log.Printf("t9\n")
	h.readFixedResponse(1) // lf

	distance := []int{}
	intensity := []int{}
	count := 1
	measure := []byte{}

	for _, v := range data {
		measure = append(measure, v)
		if len(measure) == 3 {
			if count%2 != 0 {
				distance = append(distance, decode(measure))
			} else if count%2 == 0 {
				intensity = append(intensity, decode(measure))
			}
			measure = measure[:0] // don't think too hard about this
			count++
		}
	}

	return distance, intensity, timestamp
}

func (h *HokuyoLidar) sendCommandBlock(req []byte) error {
	size := len(req)
	asize, err := h.serialPort.Write(req)
	if size != asize {
		return errors.New("Failed to send all request bytes")
	}
	return err
}

func (h *HokuyoLidar) readFixedResponse(size int) (int, []byte, error) {
	res := make([]byte, size)
	read, err := h.serialPort.Read(res)
	if read != size {
		return read, nil, errors.New("Failed to read all expected bytes")
	}
	if err != nil {
		return 0, nil, errors.New("Failed to read from serial port")
	}
	return read, res, err
}

func statusCheck(code string) {
	if code == "00" || code == "99" {
		return
	} else if _, ok := healthStatus[code]; ok {
		log.Fatalf("A known error has occured: %v: %v\n", code, healthStatus[code])
	} else {
		log.Fatalf("An unknown error has occured: %v\n", code)
	}
}

func zeroPadString(desiredLen int, str *string) {
	l := len(*str)
	if l > desiredLen {
		*str = (*str)[0:desiredLen] // trim the string to the desired length
	} else {
		for {
			if desiredLen != len(*str) {
				*str = "0" + (*str)
			} else {
				break
			}
		}
	}
}

func decode(encoded []byte) int {
	decode := 0
	for _, v := range encoded {
		v = v - 0x30
		decode <<= 6
		decode |= int(v)
	}
	return decode
}
