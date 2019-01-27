package gohokuyolidar

import (
	"errors"
	"log"

	"github.com/google/gousb"
)

// sudo chmod a+rw /dev/ttyACM0
const (
	lr          byte = 0x0a
	cr          byte = 0x0d
	version     byte = 0x56
	segmentSize int  = 65
)

// HokuyoLidar represents the lidar structure
type HokuyoLidar struct {
	context   *gousb.Context
	device    *gousb.Device
	done      func()
	out       *gousb.OutEndpoint
	in        *gousb.InEndpoint
	Connected bool
}

// NewHokuyoLidar creates an instance of the lidar struct
func NewHokuyoLidar() *HokuyoLidar {
	return &HokuyoLidar{nil, nil, nil, nil, nil, false}
}

// Connect activates the serial port connection to the lidar
func (h *HokuyoLidar) Connect() error {
	if h.Connected {
		err := errors.New("Lidar is already connected")
		return err
	}

	ctx := gousb.NewContext()
	dev, err := ctx.OpenDeviceWithVIDPID(0x15d1, 0x0000)
	if err != nil {
		log.Fatalf("Failed to open device: %v\n", err)
	}

	intf, done, err := dev.DefaultInterface()
	if err != nil {
		log.Fatalf("%s.DefaultInterface(): %v\n", dev, err)
	}

	epout, err := intf.OutEndpoint(7)
	if err != nil {
		log.Fatalf("%s.OutEndpoint(7): %v\n", intf, err)
	}

	epin, err := intf.InEndpoint(6)
	if err != nil {
		log.Fatalf("%s.InEndpoint(6): %v\n", intf, err)
	}

	h.context = ctx
	h.device = dev
	h.done = done
	h.in = epin
	h.out = epout

	h.Connected = true
	return nil
}

// Disconnect disables the serial port connection to the lidar
func (h *HokuyoLidar) Disconnect() error {
	if h.Connected {
		return errors.New("Lidar is already connected")
	}
	h.context.Close()
	h.device.Close()
	h.done()
	h.Connected = false
	return nil
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
	asize, err := h.out.Write(req)
	if size != asize {
		return errors.New("Failed to send all request bytes")
	}
	return err
}

func (h *HokuyoLidar) readUnfixedResponse(size int) (int, []byte, error) {
	res := make([]byte, size)
	read, err := h.in.Read(res)
	if err != nil {
		return 0, nil, errors.New("Failed to read from serial port")
	}
	return read, res, err
}
