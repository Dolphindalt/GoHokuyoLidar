package gohokuyolidar

import (
	"errors"

	serial "github.com/mikepb/go-serial"
)

const (
	lr          byte = 0x0a
	cr          byte = 0x0d
	version     byte = 0x56
	segmentSize int  = 65
)

// HokuyoLidar represents the lidar structure
type HokuyoLidar struct {
	serialPort *serial.Port
	portname   string
	baudrate   int // 750000
	options    *serial.Options
	Connected  bool
}

// NewHokuyoLidar creates an instance of the lidar struct
func NewHokuyoLidar(portName string, baudrate int) *HokuyoLidar {
	return &HokuyoLidar{nil, portName, baudrate, nil, false}
}

// Connect activates the serial port connection to the lidar
func (h *HokuyoLidar) Connect() error {
	if h.serialPort != nil && h.Connected {
		err := errors.New("Lidar is already connected")
		return err
	}
	options := serial.RawOptions
	options.Mode = serial.MODE_READ_WRITE
	options.BitRate = h.baudrate
	serialPort, err := options.Open(h.portname)
	if err != nil {
		err = errors.New("Failed to open serial port on " + h.portname)
		return err
	}
	h.options = &options
	h.serialPort = serialPort
	h.Connected = true
	return nil
}

// Disconnect disables the serial port connection to the lidar
func (h *HokuyoLidar) Disconnect() error {
	if h.serialPort == nil || !h.Connected {
		return nil
	}
	err := h.serialPort.Close()
	h.Connected = false
	return err
}

// VersionCmd asks the lidar for it's version info
func (h *HokuyoLidar) VersionCmd() (string, error) {
	cmd := make([]byte, 2)
	cmd[0] = version
	cmd[1] = lr
	h.sendCommandBlock(cmd)
	read, bytes, err := h.readUnfixedResponse(7 * segmentSize)
	if err != nil {
		return "", err
	}
	msg := string(bytes[:read])
	return msg, nil
}

func (h *HokuyoLidar) sendCommandBlock(req []byte) error {
	size := len(req)
	asize, err := h.serialPort.Write(req)
	if size != asize {
		return errors.New("Failed to send all request bytes")
	}
	return err
}

func (h *HokuyoLidar) readUnfixedResponse(size int) (int, []byte, error) {
	res := make([]byte, size)
	read, err := h.serialPort.Read(res)
	if err != nil {
		return 0, nil, errors.New("Failed to read from serial port")
	}
	return read, res, err
}
