package routing

import (
	"io/ioutil"
	"log"

	"gopkg.in/yaml.v2"
)

type MapYaml struct {
	Image          string     `yaml:"image"`
	FreeThresh     float64    `yaml:"free_thresh"`
	OccupiedThresh float64    `yaml:"occupied_thresh"`
	Origin         [3]float64 `yaml:"origin"`
	Resolution     float64    `yaml:"resolution"`
	Negate         float64    `yaml:"negate"`
}

func ReadImageYaml(filename string) MapYaml {
	buf, err := ioutil.ReadFile(filename)
	if err != nil {
		log.Print(err)
	}

	data := MapYaml{}

	err2 := yaml.Unmarshal(buf, &data)
	if err2 != nil {
		log.Print(err2)
	}
	return data
}
