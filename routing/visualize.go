package routing

import (
	"encoding/csv"
	"log"
	"os"
	"strconv"
)

func SaveRouteCsv(fname string, route [][3]float64) {
	file, err := os.OpenFile(fname, os.O_WRONLY|os.O_CREATE, 0600)
	if err != nil {
		log.Print(err)
	}
	defer file.Close()

	writer := csv.NewWriter(file)
	writer.Write([]string{"timestamp", "x", "y"})
	for _, r := range route {
		ts := strconv.FormatFloat(r[0], 'f', 4, 64)
		xs := strconv.FormatFloat(r[1], 'f', 4, 64)
		ys := strconv.FormatFloat(r[2], 'f', 4, 64)

		writer.Write([]string{ts, xs, ys})
	}
	writer.Flush()
}

func Convert2DPoint(obj [][2]float64) (obj2d [][]float64) {
	var xs []float64
	var ys []float64
	for _, o := range obj {
		xs = append(xs, o[0])
		ys = append(ys, o[1])
	}
	obj2d = append(obj2d, xs)
	obj2d = append(obj2d, ys)
	return obj2d
}

func Convert32DPoint(obj [][3]float64) (obj2d [][]float64) {
	var xs []float64
	var ys []float64
	for _, o := range obj {
		xs = append(xs, o[1])
		ys = append(ys, o[2])
	}
	obj2d = append(obj2d, xs)
	obj2d = append(obj2d, ys)
	return obj2d
}

func Convert3DPoint(obj [][3]float64) (obj3d [][]float64) {
	var xs []float64
	var ys []float64
	var ts []float64
	for _, o := range obj {
		xs = append(xs, o[1])
		ys = append(ys, o[2])
		ts = append(ts, o[0])
	}
	obj3d = append(obj3d, xs)
	obj3d = append(obj3d, ys)
	obj3d = append(obj3d, ts)
	return obj3d
}

func Convert23DPoint(obj [][2]float64) (obj3d [][]float64) {
	var xs []float64
	var ys []float64
	var ts []float64
	for _, o := range obj {
		xs = append(xs, o[0])
		ys = append(ys, o[1])
		ts = append(ts, 0)
	}
	obj3d = append(obj3d, xs)
	obj3d = append(obj3d, ys)
	obj3d = append(obj3d, ts)
	return obj3d
}

func (m GridMap) ConvertObjMap2Point() (obj2d [][]float64) {
	var xs []float64
	var ys []float64
	for j := 0; j < m.Height; j++ {
		for i := 0; i < m.Width; i++ {
			if m.ObjectMap[newIndex(i, j)] {
				xs = append(xs, m.Origin.X+m.Resolution*float64(i))
				ys = append(ys, m.Origin.Y+m.Resolution*float64(j))
			}
		}
	}
	obj2d = append(obj2d, xs)
	obj2d = append(obj2d, ys)
	return obj2d
}

func (m GridMap) ConvertObjMap3Point() (obj3d [][]float64) {
	var xs []float64
	var ys []float64
	var ts []float64
	for j := 0; j < m.Height; j++ {
		for i := 0; i < m.Width; i++ {
			if m.ObjectMap[newIndex(i, j)] {
				xs = append(xs, m.Origin.X+m.Resolution*float64(i))
				ys = append(ys, m.Origin.Y+m.Resolution*float64(j))
				ts = append(ts, 0)
			}
		}
	}
	obj3d = append(obj3d, xs)
	obj3d = append(obj3d, ys)
	obj3d = append(obj3d, ts)
	return obj3d
}

func (m GridMap) ConvertObjMap2PointHexa() (obj2d [][]float64) {
	var xs []float64
	var ys []float64
	for j := 0; j < m.Height; j++ {
		for i := 0; i < m.Width; i++ {
			if m.ObjectMap[newIndex(i, j)] {
				a := m.Origin.X + m.Resolution*float64(i)
				b := m.Origin.Y + m.Resolution*float64(j)
				xs = append(xs, getXAB(a, b))
				ys = append(ys, getYAB(a, b))
			}
		}
	}
	obj2d = append(obj2d, xs)
	obj2d = append(obj2d, ys)
	return obj2d
}

func (m GridMap) ConvertObjMap3PointHexa() (obj3d [][]float64) {
	var xs []float64
	var ys []float64
	var ts []float64
	for j := 0; j < m.Height; j++ {
		for i := 0; i < m.Width; i++ {
			if m.ObjectMap[newIndex(i, j)] {
				a := m.Origin.X + m.Resolution*float64(i)
				b := m.Origin.Y + m.Resolution*float64(j)
				xs = append(xs, getXAB(a, b))
				ys = append(ys, getYAB(a, b))
				ts = append(ts, 0)
			}
		}
	}
	obj3d = append(obj3d, xs)
	obj3d = append(obj3d, ys)
	obj3d = append(obj3d, ts)
	return obj3d
}
