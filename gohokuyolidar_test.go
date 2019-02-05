package gohokuyolidar

import (
	"log"
	"testing"
)

func TestStringZeroPadding(t *testing.T) {
	s := "10"
	zeroPadString(4, &s)
	if s != "0010" {
		t.Fatalf("Expected 0010, got %v in zero padding\n", s)
	}
	s = "555555"
	zeroPadString(4, &s)
	if s != "5555" {
		t.Fatalf("Expected 5555, got %v in zero padding\n", s)
	}
	s = "413921"
	zeroPadString(8, &s)
	if s != "00413921" {
		t.Fatalf("Expected 00413921, got %v in zero padding\n", s)
	}
}

func TestDecode(t *testing.T) {
	b := []byte{0x6D, 0x32, 0x40, 0x30}
	val := decode(b)
	if val != 16000000 {
		log.Fatalf("Expected to decode 16000000, got %v instead\n", val)
	}
}
