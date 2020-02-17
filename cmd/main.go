package main

import (
	"bufio"
	"can-parser/internal/readCanMsgs"
	"fmt"
	"github.com/360EntSecGroup-Skylar/excelize"
	"strconv"
	//"reflect"
	"log"
	"os"
	"strings"
)

type CanBus struct {
	timeStamp string
	id        uint32
	payload   [8]uint8
}

func main() {

	c := &readCanMsgs.CANSensors{
		GPS:          &readCanMsgs.GPS{},
		Inclinometer: &readCanMsgs.Inclinometer{},
		EncoderL:     &readCanMsgs.EncoderL{Counts: 0, LastCounts: 0, Direction: readCanMsgs.NotMoving},
		EncoderR:     &readCanMsgs.EncoderR{Counts: 0, LastCounts: 0, Direction: readCanMsgs.NotMoving},
	}

	var msg CanBus

	numLine := 1

	filePathName := "test_inclino"
	file, err := os.Open("../can-msgs/" + filePathName)
	if err != nil {
		log.Fatal(err)
	}
	defer file.Close()

	scanner := bufio.NewScanner(file)

	excelFile, err := excelize.OpenFile("../template.xlsx")
	if err != nil {
		fmt.Println(err)
	}

	excelFile.SetActiveSheet(1)
	startRow := 2
	fmt.Println("Parsing is running...")
	for scanner.Scan() {

		if numLine >= 4 {

			line := scanner.Text()
			msg.timeStamp = line[:11]

			id, err := strconv.ParseUint(line[15:18], 16, 64)
			if err != nil {
				//fmt.Println(err)
			}

			msg.id = uint32(id)

			//msg.data[d].id = line[15:18]
			strPayload := line[39:]
			strPayloadPure := strings.Fields(strPayload) // remove spaces spaces
			var dataArray [8]uint8
			for i, element := range strPayloadPure {

				dataElement, err := strconv.ParseUint(element, 16, 8)
				if err != nil {
					fmt.Println(err)
				}
				dataArray[i] = uint8(dataElement)

			}

			msg.payload = dataArray

			//fmt.Printf("%X\n", dataArray)
			//fmt.Printf("%X\n", id)

			startRow = readCanMsgs.ReadCanMsgs(msg.payload, msg.id, msg.timeStamp, startRow, excelFile, c)

			//d++
			//startRow++

		}

		//if numLine >= 10000{
		//	break
		//}
		numLine++

	}

	logPath := "../logs/log_" + filePathName + ".xlsx"
	err = excelFile.SaveAs(logPath)
	if err != nil {
		fmt.Println(err)
	}
	fmt.Println("Parsing is done...")

}
