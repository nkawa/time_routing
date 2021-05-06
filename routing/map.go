package routing

import (
	"image"
	"image/color"
	"log"
	"os"

	ros "github.com/fukurin00/go_ros_msg"
	_ "github.com/jbuchbinder/gopnm"
)

type MapMeta struct {
	W      int
	H      int
	Origin Point
	Reso   float64
	Data   []int8
}

func LoadROSMap(grid ros.OccupancyGrid, closeThreth int) *MapMeta {
	m := new(MapMeta)
	m.H = int(grid.Info.Height)
	m.W = int(grid.Info.Width)
	m.Origin = Point{X: grid.Info.Origin.Position.X, Y: grid.Info.Origin.Position.Y}
	m.Reso = float64(grid.Info.Resolution)
	m.Data = grid.Data
	return m
}

// read image file of ROS format
func ReadStaticMapImage(yamlFile, mapFile string, closeThreth float64) (*MapMeta, error) {
	m := new(MapMeta)
	mapConfig := ReadImageYaml(yamlFile)
	m.Reso = mapConfig.Resolution
	m.Origin = Point{X: mapConfig.Origin[0], Y: mapConfig.Origin[1]}

	file, err := os.Open(mapFile)
	if err != nil {
		return m, err
	}
	defer file.Close()

	imageData, _, err := image.Decode(file)
	if err != nil {
		return m, err
	}

	bound := imageData.Bounds()
	m.W = bound.Dx()
	m.H = bound.Dy()

	data := make([]int8, m.W*m.H)
	open := 0
	close := 0
	for j := m.H - 1; j >= 0; j-- {
		for i := 0; i < m.W; i++ {
			oldPix := imageData.At(i, j)
			pixel := color.GrayModel.Convert(oldPix)
			pixelU := color.GrayModel.Convert(pixel).(color.Gray).Y

			a := (255.0 - float64(pixelU)) / 255.0
			var v int8 = 0
			if a > closeThreth {
				v = 100
				close += 1
			} else {
				v = 0
				open += 1
			}
			data[i+(m.H-j-1)*m.W] = v
		}
	}
	log.Printf("open: %d, close: %d", open, close)
	m.Data = data

	return m, nil
}

func (m MapMeta) GetObjectMap() [][2]float64 {
	var objMap [][2]float64
	insideWall := false
	for i, pixel := range m.Data {

		if pixel >= 90 {
			if insideWall {
				continue
			}
			x := float64(i%m.W)*(m.Reso) + (m.Origin.X)
			y := float64(i/m.W)*(m.Reso) + (m.Origin.Y)
			var oPoint = [2]float64{x, y}
			objMap = append(objMap, oPoint)
		} else {
			insideWall = false
		}
	}
	return objMap
}
