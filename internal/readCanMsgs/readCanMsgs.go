package readCanMsgs

import (
	"encoding/binary"
	"github.com/360EntSecGroup-Skylar/excelize"
	"math"
	"strconv"
	"time"
)

type CANSensors struct {
	Sensors      []Sensor `json:"sensors"`
	GPS          *GPS
	Inclinometer *Inclinometer
	EncoderL     *EncoderL
	EncoderR     *EncoderR
}

type Sensor struct {
	Id     string      `json:"Id"`
	Sample interface{} `json:"sample"`
}

// Encoder settings

type EncoderL struct {
	Counts                int64
	Distance              float32
	VehicleSpeed          float32
	LastCounts            uint32
	Direction             string
	EstimatedFrequency    float64
	CurrentTime           time.Time
	LastSampleTime        time.Time
	LastTriggeredDistance float32
}

type EncoderR struct {
	Counts                int64
	Distance              float32
	VehicleSpeed          float32
	LastCounts            uint32
	Direction             string
	EstimatedFrequency    float64
	CurrentTime           time.Time
	LastSampleTime        time.Time
	LastTriggeredDistance float32
}

type TruckSpecifications struct {
	EncoderWheelRadius            float32 `json:"encoderWheelRadius"`            //meters
	TruckOuterWheel               float32 `json:"truckOuterWheel"`               //meters
	TruckInnerWheel               float32 `json:"truckInnerWheel"`               //meters
	WheelBase                     float32 `json:"wheelBase"`                     //meters
	PotentiometerMountingDistance float32 `json:"potentiometerMountingDistance"` //meters
}

const (
	ClockWise        = "CLOCK_WISE"
	CounterClockWise = "COUNTER_CLOCK_WISE"
	NotMoving        = "NOT_MOVING"
)

const (
	MaximumCounts               uint32 = 0xFFFFFFFF
	RammingDetectionThreshold          = 0x00040000
	DirectionDetectionThreshold        = 0x00000040
	CountsPerRevolution                = 0xFFFF
)

const (
	EncoderNodeNameLeft uint8  = 0x3F
	EncoderCountsLeft   uint32 = 0x1BF
	EncoderSpeedLeft    uint32 = 0x3BF
)

const (
	EncodeNodeNameRight uint8  = 0x3A
	EncoderCountsRight  uint32 = 0x1BA
	EncoderSpeedRight   uint32 = 0x3BA
)

// GPS settings

type GPS struct {
	Altitude           float32
	Longitude          float32
	Latitude           float32
	PDOP               string
	Speed              float32
	AntennaStatus      string
	NavigationMethod   string
	CurrentTime        time.Time
	LastSampleTime     time.Time
	EstimatedFrequency float64
}

//gps msgs ids
const (
	GPSLongitude   uint32 = 0x622
	GPSLatitude    uint32 = 0x623
	GPSAltitude    uint32 = 0x624
	GPSPositionDOP uint32 = 0x625
	GPSSpeed       uint32 = 0x621
	GPSStatus      uint32 = 0x620
)

//directions
const (
	North string = "N"
	South string = "S"
	East  string = "E"
	West  string = "W"
)

//Antenna STATUS
const (
	INIT     string = "INIT"
	DONTKNOW string = "DONTKNOW"
	OK       string = "OK"
	SHORT    string = "SHORT"
	OPEN     string = "OPEN"
)

//navigation method
const (
	NONE   string = "NONE"
	TwoD   string = "2D"
	ThreeD string = "3D"
)

//signal quality
const (
	IDEAL     string = "IDEAL"
	EXCELLENT string = "EXCELLENT"
	GOOD      string = "GOOD"
	MODERATE  string = "MODERATE"
	FAIR      string = "FAIR"
	POOR      string = "POOR"
)

// inclinometer settings

type Inclinometer struct {
	LongSlope    float32
	LateralSlope float32
}

//messages ID
const (
	InclinometerNewNodeName uint8  = 0x0A
	InclinometerSlopesNew   uint32 = 0x28A
)

const (
	Gyro         uint32 = 0x2CD
	Acceleration uint32 = 0x1CD
)

const (
	InclinometerOldNodeName uint8  = 0x3E
	InclinometerSlopesOld   uint32 = 0x1BE
)

//excelSheet Cols
const (
	TimeStampsCol      int = 1
	DistanceLeftCol    int = 2
	SpeedLeftCol       int = 3
	DistanceRightCol   int = 4
	SpeedRightCol      int = 5
	LongSlopeNewCol    int = 6
	LateralSlopeNewCol int = 7
	LongSlopeOldCol    int = 8
	LateralSlopeOldCol int = 9
	LongitudeCol       int = 10
	LatitudeCol        int = 11
	AltitudeCol        int = 12
	GPSPositionCol     int = 13
	GPSStatusCol       int = 14
	GPSSpeedCol        int = 15
	GyroXAxisCol       int = 16
	GyroYAxisCol       int = 17
	GyroZAxisCol       int = 18
	AccXAxisCol        int = 19
	AccYAxisCol        int = 20
	AccZAxisCol        int = 21
)

func ReadCanMsgs(data [8]uint8, id uint32, timestamp string, startRow int, f *excelize.File, d *CANSensors) (newRow int) {

	var truckSpec TruckSpecifications
	truckSpec.EncoderWheelRadius = 0.08
	truckSpec.PotentiometerMountingDistance = 1.5
	truckSpec.TruckInnerWheel = 0.2
	truckSpec.TruckOuterWheel = 0.36
	truckSpec.PotentiometerMountingDistance = 1.5
	truckSpec.WheelBase = 1.5

	directions := make([]string, 2)
	var sign float32
	switch id {
	case GPSLongitude:

		minutesRaw := data[0:4]
		degreesRaw := data[4:6]
		indicator := data[6:7]
		switch indicator[0] {
		case 69:
			sign = 1
			directions[1] = East
		case 87:
			sign = -1
			directions[1] = West
		}
		degreesValue := binary.LittleEndian.Uint16(degreesRaw)
		minutesBits := binary.LittleEndian.Uint32(minutesRaw)
		minutesValue := math.Float32frombits(minutesBits)
		longitude := sign * (float32(degreesValue) + minutesValue/60.0)

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, LongitudeCol), longitude)
		return startRow + 1

	case GPSLatitude:
		minutesRaw := data[0:4]
		degreesRaw := data[4:6]
		indicator := data[6:7]

		switch indicator[0] {
		case 78:
			sign = 1
			directions[0] = North
		case 83:
			sign = -1
			directions[0] = South
		}

		degreesValue := binary.LittleEndian.Uint16(degreesRaw)
		minutesBits := binary.LittleEndian.Uint32(minutesRaw)
		minutesValue := math.Float32frombits(minutesBits)
		latitude := float32(degreesValue) + minutesValue/60.0

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, LatitudeCol), latitude)
		return startRow + 1

	case GPSAltitude:
		altitudeRaw := data[0:4]
		altitudeBits := binary.LittleEndian.Uint32(altitudeRaw)
		altitude := math.Float32frombits(altitudeBits)

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, AltitudeCol), altitude)
		return startRow + 1

	case GPSPositionDOP:
		positionDOPRaw := data[0:4]
		positionDOPBits := binary.LittleEndian.Uint32(positionDOPRaw)
		positionDOP := math.Float32frombits(positionDOPBits)
		switch p := positionDOP; {
		case p == 1:
			d.GPS.PDOP = IDEAL
		case p < 2:
			d.GPS.PDOP = EXCELLENT
		case p < 5:
			d.GPS.PDOP = GOOD
		case p < 10:
			d.GPS.PDOP = MODERATE
		case p < 20:
			d.GPS.PDOP = FAIR
		case p > 20:
			d.GPS.PDOP = POOR
		}
		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, GPSPositionCol), d.GPS.PDOP)
		return startRow + 1
	case GPSStatus:
		antennaStatusValue := uint(data[0])
		switch antennaStatusValue {
		case 0:
			d.GPS.AntennaStatus = INIT
		case 1:
			d.GPS.AntennaStatus = DONTKNOW
		case 2:
			d.GPS.AntennaStatus = OK
		case 3:
			d.GPS.AntennaStatus = SHORT
		case 4:
			d.GPS.AntennaStatus = OPEN
		}
		navigationMethodValue := uint(data[2])
		switch navigationMethodValue {
		case 0:
			d.GPS.NavigationMethod = INIT
		case 1:
			d.GPS.NavigationMethod = NONE
		case 2:
			d.GPS.NavigationMethod = TwoD
		case 3:
			d.GPS.NavigationMethod = ThreeD
		}
		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, GPSStatusCol), d.GPS.AntennaStatus)
		return startRow + 1

	case GPSSpeed:
		speedRaw := data[4:8]
		speedBits := binary.LittleEndian.Uint32(speedRaw)
		speed := math.Float32frombits(speedBits)
		d.GPS.Speed = speed
		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, GPSSpeedCol), speed)
		return startRow + 1

	case InclinometerSlopesNew:

		longSlopeRaw := data[0:2]
		lateralSlopeRaw := data[2:4]
		longSlopeBits := binary.LittleEndian.Uint16(longSlopeRaw)
		lateralSlopeBits := binary.LittleEndian.Uint16(lateralSlopeRaw)
		longSlope := float32(int16(longSlopeBits)) * 0.01
		lateralSlope := float32(int16(lateralSlopeBits)) * 0.01

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, LongSlopeNewCol), longSlope)
		f.SetCellValue("sheet1", cell(startRow, LateralSlopeNewCol), lateralSlope)
		return startRow + 1

	case InclinometerSlopesOld:

		longSlopeRaw := data[0:2]
		lateralSlopeRaw := data[2:4]
		longSlopeBits := binary.LittleEndian.Uint16(longSlopeRaw)
		lateralSlopeBits := binary.LittleEndian.Uint16(lateralSlopeRaw)
		longSlope := float32(int16(longSlopeBits)) * 0.01
		lateralSlope := float32(int16(lateralSlopeBits)) * 0.01

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, LongSlopeOldCol), longSlope)
		f.SetCellValue("sheet1", cell(startRow, LateralSlopeOldCol), lateralSlope)
		return startRow + 1
	case Gyro:

		GyroXAxisRaw := data[0:2]
		GyroYAxisRaw := data[2:4]
		GyroZAxisRaw := data[4:6]

		GyroXAxisBits := binary.LittleEndian.Uint16(GyroXAxisRaw)
		GyroYAxisBits := binary.LittleEndian.Uint16(GyroYAxisRaw)
		GyroZAxisBits := binary.LittleEndian.Uint16(GyroZAxisRaw)

		GyroXAxis := float32(int16(GyroXAxisBits)) * math.Pi / 180 // from degrees/s  to rad /sec
		GyroYAxis := float32(int16(GyroYAxisBits)) * math.Pi / 180
		GyroZAxis := float32(int16(GyroZAxisBits)) * math.Pi / 180

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, GyroXAxisCol), GyroXAxis)
		f.SetCellValue("sheet1", cell(startRow, GyroYAxisCol), GyroYAxis)
		f.SetCellValue("sheet1", cell(startRow, GyroZAxisCol), GyroZAxis)

		return startRow + 1
	case Acceleration:

		AccXAxisRaw := data[0:2]
		AccYAxisRaw := data[2:4]
		AccZAxisRaw := data[4:6]

		AccXAxisBits := binary.LittleEndian.Uint16(AccXAxisRaw)
		AccYAxisBits := binary.LittleEndian.Uint16(AccYAxisRaw)
		AccZAxisBits := binary.LittleEndian.Uint16(AccZAxisRaw)

		AccXAxis := float32(int16(AccXAxisBits)) * 9.81 * 0.001 // from mg to m/s2
		AccYAxis := float32(int16(AccYAxisBits)) * 9.81 * 0.001
		AccZAxis := float32(int16(AccZAxisBits)) * 9.81 * 0.001

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, AccXAxisCol), AccXAxis)
		f.SetCellValue("sheet1", cell(startRow, AccYAxisCol), AccYAxis)
		f.SetCellValue("sheet1", cell(startRow, AccZAxisCol), AccZAxis)

		return startRow + 1
	case EncoderCountsLeft:

		actualCounts := data[0:4]
		actualCountsBits := binary.LittleEndian.Uint32(actualCounts)
		diffCounts := int64(actualCountsBits) - int64(d.EncoderL.LastCounts)

		diffCountsFloat := float64(diffCounts)

		switch diff := diffCounts; {
		case diff > DirectionDetectionThreshold:
			d.EncoderL.Direction = ClockWise
		case diff < -DirectionDetectionThreshold:
			d.EncoderL.Direction = CounterClockWise
		default:
			d.EncoderL.Direction = NotMoving
		}

		if math.Abs(diffCountsFloat) > RammingDetectionThreshold {
			if diffCountsFloat < 0 {
				d.EncoderL.Counts += int64(MaximumCounts) - int64(d.EncoderL.LastCounts) + int64(actualCountsBits)
				d.EncoderL.Direction = ClockWise
			} else {
				d.EncoderL.Counts += int64(actualCountsBits) - int64(MaximumCounts) - int64(d.EncoderL.LastCounts)
				d.EncoderL.Direction = CounterClockWise
			}
		} else {
			d.EncoderL.Counts += diffCounts
		}

		distance := (float32(d.EncoderL.Counts) / float32(CountsPerRevolution)) * math.Pi * 2 * truckSpec.EncoderWheelRadius

		d.EncoderL.Distance = distance
		d.EncoderL.LastCounts = actualCountsBits

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, DistanceLeftCol), distance)

		return startRow + 1

	case EncoderSpeedLeft:
		speedRaw := data[0:4]
		speedBits := binary.LittleEndian.Uint16(speedRaw)
		speed := int16(speedBits) // rev/min
		vehicleSpeed := (2 * math.Pi * float32(speed) / 60) * truckSpec.EncoderWheelRadius * 3.6
		d.EncoderL.VehicleSpeed = vehicleSpeed

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, SpeedLeftCol), vehicleSpeed)
		return startRow + 1

	case EncoderCountsRight:

		actualCounts := data[0:4]
		actualCountsBits := binary.LittleEndian.Uint32(actualCounts)
		diffCounts := int64(actualCountsBits) - int64(d.EncoderR.LastCounts)

		diffCountsFloat := float64(diffCounts)

		switch diff := diffCounts; {
		case diff > DirectionDetectionThreshold:
			d.EncoderR.Direction = ClockWise
		case diff < -DirectionDetectionThreshold:
			d.EncoderR.Direction = CounterClockWise
		default:
			d.EncoderR.Direction = NotMoving
		}

		if math.Abs(diffCountsFloat) > RammingDetectionThreshold {
			if diffCountsFloat < 0 {
				d.EncoderR.Counts += int64(MaximumCounts) - int64(d.EncoderR.LastCounts) + int64(actualCountsBits)
				d.EncoderR.Direction = ClockWise
			} else {
				d.EncoderR.Counts += int64(actualCountsBits) - int64(MaximumCounts) - int64(d.EncoderR.LastCounts)
				d.EncoderR.Direction = CounterClockWise
			}
		} else {
			d.EncoderR.Counts += diffCounts
		}

		distance := (float32(d.EncoderR.Counts) / float32(CountsPerRevolution)) * math.Pi * 2 * truckSpec.EncoderWheelRadius

		d.EncoderR.LastCounts = actualCountsBits

		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, DistanceRightCol), distance)
		return startRow + 1

	case EncoderSpeedRight:
		speedRaw := data[0:4]
		speedBits := binary.LittleEndian.Uint16(speedRaw)
		speed := int16(speedBits)                                                                // rev/min
		vehicleSpeed := (2 * math.Pi * float32(speed) / 60) * truckSpec.EncoderWheelRadius * 3.6 //km/h
		d.EncoderR.VehicleSpeed = vehicleSpeed
		f.SetCellValue("sheet1", cell(startRow, TimeStampsCol), timestamp)
		f.SetCellValue("sheet1", cell(startRow, SpeedRightCol), vehicleSpeed)
		return startRow + 1
	}
	return startRow
}

func cell(i int, j int) string {
	row := strconv.Itoa(i)
	column := string('A' - 1 + j)
	return column + row
}
