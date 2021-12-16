package routing

import (
	"encoding/csv"
	"log"
	"os"
	"strconv"
	"time"
)

func SaveRouteCsv(fname string, times []time.Time, route [][2]float64) {
	file, err := os.OpenFile(fname, os.O_WRONLY|os.O_CREATE, 0600)
	if err != nil {
		log.Print(err)
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	writer.Write([]string{"t", "x", "y"})
	for i, r := range route {
		ts := strconv.FormatInt(times[i].UnixNano(), 10)
		xs := strconv.FormatFloat(r[0], 'f', 4, 64)
		ys := strconv.FormatFloat(r[1], 'f', 4, 64)

		writer.Write([]string{ts, xs, ys})
	}
	writer.Flush()
}
