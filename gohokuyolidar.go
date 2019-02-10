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
	mTag          byte = 0x4d
	bTag          byte = 0x42
	qTag          byte = 0x51
	tTag          byte = 0x54
	rTag          byte = 0x52
	sTag          byte = 0x53
	cTag          byte = 0x43
	hTag          byte = 0x48
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

// NewHokuyoLidar creates an instance of the lidar struct.
func NewHokuyoLidar(portName string, baudrate int) *HokuyoLidar {
	return &HokuyoLidar{nil, portName, baudrate, false, nil, false, false,
		0, 0, 0, 0, 0, 0, 0}
}

// Connect activates the serial port connection to the lidar.
// Some devices run scip 1.1 by default. If so, specify scip1IsDefault as true.
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

// Disconnect disables the serial port connection to the lidar.
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

// MDMSCmd is a sensor data aquisition command that uses three character encoding or two character encoding.
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
	if len(characters) > 16 {
		characters = characters[0:16]
	}

	var encode byte
	if three {
		encode = threeEncoding
	} else {
		encode = twoEncoding
	}

	cmd := []byte{mTag, encode}
	cmd = append(cmd[:], []byte(ss)[:]...)
	cmd = append(cmd[:], []byte(es)[:]...)
	cmd = append(cmd[:], []byte(cc)[:]...)
	cmd = append(cmd[:], []byte(si)[:]...)
	cmd = append(cmd[:], []byte(ns)[:]...)
	cmd = append(cmd[:], []byte(characters)[:]...)
	cmd = append(cmd[:], lf)

	err := h.sendCommandBlock(cmd)
	if err != nil {
		log.Fatalf("Encountered error during MD init: %v\n", err)
	}
	headLen := 21 + len(characters)
	_, head, err := h.readFixedResponse(headLen)
	if err != nil {
		log.Fatalf("Err in scan init: %v\n", err)
	}
	statusCode := head[headLen-5 : headLen-3]
	statusCheck(string(statusCode))

	h.startStep = startStep
	h.endStep = endStep
	h.clusterCount = clusterCount
	h.scanInterval = scanInterval
	h.encodingType = threeEncoding
	h.headSize = headLen
	h.requestTag = mTag
}

// BMCommand will illuminate the sensor’s laser enabling the measurement.
// When sensor is switched on in SCIP2.0 mode or switched to SCIP2.0 by
// command the laser is initially in off state by default. In this state
// sensor can not perform the measurement. Laser state can be verified
// by green LED on the sensor. Laser is off if the LED blinks rapidly
// and it is ON when LED glows continuously.
func (h *HokuyoLidar) BMCommand(chars string) error {
	cmd := []byte{bTag, mTag}
	cmd = append(cmd[:], []byte(chars)[:]...)
	cmd = append(cmd, lf)
	err := h.sendCommandBlock(cmd)
	if err != nil {
		return err
	}
	resLen := len(chars) + 8
	_, res, err := h.readFixedResponse(resLen)
	if err != nil {
		return err
	}
	statusCode := res[resLen-5 : resLen-3]
	statusCheck(string(statusCode))
	return err
}

// QMCommand will switch off the laser disabling sensor’s measurement state.
func (h *HokuyoLidar) QMCommand(chars string) error {
	cmd := []byte{qTag, tTag}
	cmd = append(cmd[:], []byte(chars)[:]...)
	cmd = append(cmd, lf)
	err := h.sendCommandBlock(cmd)
	if err != nil {
		return err
	}
	resLen := len(chars) + 8
	_, _, err = h.readFixedResponse(resLen) // status is always 0 0
	return err
}

// RSCommand will reset all the settings that were changed after sensor
// was switched on. This turns Laser off, sets motor speed and bit rate
// back to default as well as reset sensor’s internal timer.
func (h *HokuyoLidar) RSCommand(chars string) error {
	cmd := []byte{rTag, sTag}
	cmd = append(cmd[:], []byte(chars)[:]...)
	cmd = append(cmd, lf)
	err := h.sendCommandBlock(cmd)
	if err != nil {
		return err
	}
	_, _, err = h.readFixedResponse(len(chars) + 8) // status is always 0 0
	return err
}

// TMCommand is used to adjust (match) the host and sensor time. Sensor
// should be switched to adjust mode before requesting its time and
// mode should be switched off after the adjustment. When the sensor is
// inadjustment mode laser is switched off and it will not accept any other
// commands unless the mode is terminated. Sending multiple TM Command
// with differentstring lengths and comparing the time can estimate
// average data transmission time between sensor and host.
// Control byte: 0 -> adjust mode on, 2 -> time request, 3 -> adjust mode off.
// int will return time if control is 1, 0 if error or not 1.
func (h *HokuyoLidar) TMCommand(control byte, chars string) (int, error) {
	cmd := []byte{tTag, mTag, control}
	cmd = append(cmd[:], []byte(chars)[:]...)
	cmd = append(cmd, lf)
	err := h.sendCommandBlock(cmd)
	if err != nil {
		return 0, err
	}
	_, head, err := h.readFixedResponse(len(cmd) + 2)
	headLen := len(head)
	statusCode := string(head[headLen-2 : headLen-1])
	switch statusCode {
	case "01":
		return 0, errors.New("Invalid Control Code")
	case "02":
		return 0, errors.New("Adjust mode on when already on")
	case "03":
		return 0, errors.New("Adjust mode off when already off")
	case "04":
		return 0, errors.New("Adjust mode off when time requested")
	default:
	}
	if control == '1' {
		_, res, err := h.readFixedResponse(9)
		if err != nil {
			return 0, err
		}
		time := decode(res[2:6])
		return time, nil
	}
	_, _, err = h.readFixedResponse(3)
	return 0, err
}

// SSCommand will change the communication bit rate of the sensor
// when connected with RS232C.
// Bit Rate:
// 019200 --- 19.2 Kbps
// 038400 --- 38.4 Kbps (Some sensor models may not be compatible to this speed)
// 057600 --- 57.6 Kbps.
// 115200 --- 115.2 Kbps.
// 250000 --- 250.0 Kbps
// 500000 --- 500.0 Kbps
// 750000 --- 750.0 Kbps.
func (h *HokuyoLidar) SSCommand(sixCharacterBitRate string, chars string) error {
	if len(sixCharacterBitRate) != 6 {
		return errors.New("Invalid bitrate string")
	}
	cmd := []byte{sTag, sTag}
	cmd = append(cmd[:], []byte(sixCharacterBitRate)[:]...)
	cmd = append(cmd[:], []byte(chars)[:]...)
	cmd = append(cmd, lf)
	err := h.sendCommandBlock(cmd)
	if err != nil {
		return err
	}
	_, res, err := h.readFixedResponse(len(cmd) + 5)
	if err != nil {
		return err
	}
	statusCode := string(res[len(cmd) : len(cmd)+2])
	switch statusCode {
	case "01":
		return errors.New("Bit rate has non-numeric value")
	case "02":
		return errors.New("Invlaid bit rate")
	case "03":
		return errors.New("Sensor is already running at defined bit rate")
	case "04":
		return errors.New("Not compatible with the sensor model")
	default:
	}
	return nil
}

// HSCommand will switch between high sensitivity and normal sensitivity modes.
// Sensor’s detection ability will increase about 20% in the high sensitivity
// mode. However there may be chances of measurement errors due to strong
// reflective objects near 22m.
func (h *HokuyoLidar) HSCommand(highMode bool, chars string) error {
	var param byte
	if highMode {
		param = '1'
	} else {
		param = '0'
	}
	cmd := []byte{hTag, sTag, param}
	cmd = append(cmd[:], []byte(chars)[:]...)
	cmd = append(cmd, lf)
	err := h.sendCommandBlock(cmd)
	if err != nil {
		return err
	}
	_, res, err := h.readFixedResponse(len(cmd) + 5)
	statusCode := string(res[len(cmd) : len(cmd)+2])
	switch statusCode {
	case "01":
		return errors.New("Parameter error")
	case "02":
		return errors.New("Already running in set mode")
	case "03":
		return errors.New("Incompatible with current sensor model")
	default:
	}
	return nil
}

// CRCommand is used to adjust the sensor’s motor speed.
func (h *HokuyoLidar) CRCommand(chars string) error {
	cmd := []byte{cTag, rTag, chars[0], chars[1]}
	cmd = append(cmd[:], []byte(chars)[:]...)
	cmd = append(cmd, lf)
	err := h.sendCommandBlock(cmd)
	if err != nil {
		return err
	}
	_, res, err := h.readFixedResponse(len(cmd) + 6)
	if err != nil {
		return err
	}
	statusCode := string(res[len(cmd)+1 : len(cmd)+3])
	switch statusCode {
	case "01":
		return errors.New("Invalid speed ratio")
	case "02":
		return errors.New("Speed ratio out of range")
	case "03":
		return errors.New("Motor is already running at defined speed")
	case "04":
		return errors.New("Incompatible with current sensor model")
	default:
	}
	return nil
}

// GetDistanceAndIntensity returns a list of distances, intensities, and a timestamp
func (h *HokuyoLidar) GetDistanceAndIntensity() ([]int, []int, int) {
	var resLen int
	if h.requestTag == 'M' {
		resLen = int(h.headSize)
	} else {
		resLen = int(h.headSize - 10)
	}
	_, _, err := h.readFixedResponse(resLen)
	if err != nil {
		log.Fatalf("Failed to read reponse header: %v\n", err)
	}
	_, statusAndJunk, err := h.readFixedResponse(4)
	if err != nil {
		log.Fatalf("Failed to read status of scan: %v\n", err)
	}
	statusCode := string(statusAndJunk[0:2])
	statusCheck(statusCode)
	_, encodedTime, err := h.readFixedResponse(6)
	if err != nil {
		log.Fatalf("Failed to read timestamp: %v\n", err)
	}
	timestamp := decode(encodedTime)

	data := []byte{}
	for {
		_, chungus, err := h.readFixedResponse(66) // data plus lf lf
		if err != nil {
			log.Fatalf("Failed to read data chunk during scan: %v\n", err)
		}
		data = append(data[:], chungus[0:len(chungus)-2]...)
		dataleft := string(chungus[13:15])
		if dataleft == "00" {
			break
		}
	}

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
