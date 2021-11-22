package main

import (
	"context"
	"flag"
	"fmt"
	"io"
	"log"
	"math"
	"os"
	"strings"
	"sync"
	"time"

	ros "github.com/fukurin00/go_ros_msg"
	"github.com/fukurin00/time_routing/msg"
	"github.com/fukurin00/time_routing/robot"
	grid "github.com/fukurin00/time_routing/routing"
	"github.com/fukurin00/time_routing/synerex"

	cav "github.com/synerex/proto_cav"
	api "github.com/synerex/synerex_api"
	sxutil "github.com/synerex/synerex_sxutil"
	"google.golang.org/protobuf/proto"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	astar "github.com/fukurin00/astar_golang"
	"github.com/synerex/proto_mqtt"
)

const (
	closeThresh float64 = 0.85

	mapFile  string = "map/projection_edit.pgm"
	yamlFile string = "map/projection_edit.yaml"
)

var (
	mode Mode = ASTAR3DHEXA

	// runtime parameter
	robotsize   = flag.Float64("robotSize", 0.41, "robot radius")
	robotVel    = flag.Float64("robotVel", 0.3, "robot velocity")
	robotRotVel = flag.Float64("robotRotVel", 0.5, "robot rotation velocity")
	resolution  = flag.Float64("reso", 0.28, "path planning resolution")

	mapMetaUpdate               = false
	mapMeta       *grid.MapMeta = nil
	gridMap       *grid.GridMap = nil
	astarPlanner  *astar.Astar  //if 2d mode

	robotList map[int]*robot.RobotStatus

	timeRobotMap grid.TimeRobotMap = nil // ロボットがいるかどうかのマップ
	timeMapMin   time.Time               //time mapの最小時刻

	clt mqtt.Client

	msgCh   chan mqtt.Message
	routeCh chan *cav.DestinationRequest

	timeStep         float64 //計算に使う1stepの秒数
	reso             float64
	robotRadius      float64
	robotVelocity    float64
	robotRotVelocity float64
	aroundCell       int
)

func init() {
	msgCh = make(chan mqtt.Message)
	routeCh = make(chan *cav.DestinationRequest)

	robotList = make(map[int]*robot.RobotStatus)

	flag.Parse()
	reso = *resolution
	robotRadius = *robotsize
	robotVelocity = *robotVel
	robotRotVelocity = *robotRotVel
	timeStep = reso / robotVelocity
	//timeStep = reso/robotVelocity + 2*math.Pi*robotRadius/3/robotRotVelocity // L/v + 2pi/3w  120度回転したときの一番かかる時間
	//timeStep = float64(math.Ceil(reso/robotVelocity + 2*math.Pi*robotRadius/3/robotRotVelocity)) //切り上げ整数
	aroundCell = grid.GetAoundCell(robotRadius, reso)
}

type Mode int

const (
	ASTAR2D     Mode = iota //normal astar
	ASTAR3D                 //original astar
	ASTAR3DHEXA             //original hexa astar
)

func (m Mode) String() string {
	s := [3]string{"Astar2D", "Astar3D", "HexaAstar3d"}
	return s[m]
}

func handleRouting() {
	for {
		req := <-routeCh
		routing(req)
	}
}

func routing(rcd *cav.DestinationRequest) {
	var jsonPayload []byte
	if mode == ASTAR3DHEXA {
		if gridMap == nil {
			log.Print("not receive gridMap yet ...")
			return
		}

		// start, goal node
		isa, isb := gridMap.Pos2IndHexa(float64(rcd.Current.X), float64(rcd.Current.Y))
		iga, igb := gridMap.Pos2IndHexa(float64(rcd.Destination.X), float64(rcd.Destination.Y))

		if val, ok := robotList[int(rcd.RobotId)]; ok {
			val.SetDest(ros.Point{X: float64(rcd.Destination.X), Y: float64(rcd.Destination.Y)})
		}

		// 止まってるロボットの位置取得
		var otherCount int = 0
		others := make(map[grid.Index]bool)
		for id, robot := range robotList {
			if id == int(rcd.RobotId) {
				continue
			} else {
				if !robot.HavePath {
					otherCount += 1
					ia, ib := gridMap.Pos2IndHexa(robot.Pos.X, robot.Pos.Y)
					others[grid.NewIndex(ia, ib)] = true
					if aroundCell >= 2 {
						for _, an := range grid.Around {
							others[grid.NewIndex(ia+an[0], ib+an[1])] = true
						}
					}
				}
			}
		}
		log.Printf("robot is %d not have path robot is %d", len(robotList), otherCount)

		// update robot map
		now := time.Now()
		elap := now.Sub(timeMapMin).Seconds()
		updateStep := int(math.Round(elap / timeStep))
		timeRobotMap = gridMap.UpdateStep(timeRobotMap, updateStep)
		addTime := time.Duration(int64(float64(updateStep)*timeStep*math.Pow10(6))) * time.Microsecond
		timeMapMin = timeMapMin.Add(addTime)
		log.Print(timeMapMin)
		log.Printf("elaps %fseconds update robot cost map %dtimestep, %f added", elap, updateStep, addTime.Seconds())

		//planning
		routei, stops, err := gridMap.PlanHexa(int(rcd.RobotId), isa, isb, iga, igb, robotVelocity, robotRotVelocity, timeStep, grid.TRWCopy(timeRobotMap), others)

		if err != nil {
			log.Print(err)
		} else {
			// send path
			route := gridMap.Route2PosHexa(float64(timeMapMin.UnixNano())*float64(math.Pow10(-9)), timeStep, routei)
			jsonPayload, err = msg.MakePathMsg(route)
			if err != nil {
				log.Print(err)
			}
			sendPath(jsonPayload, int(rcd.RobotId))

			// update
			gridMap.UpdateTimeObjMapHexa(timeRobotMap, routei, aroundCell)
			now := time.Now()
			elap := now.Sub(timeMapMin).Seconds()
			updateStep := int(math.Round(elap / timeStep))
			timeRobotMap = gridMap.UpdateStep(timeRobotMap, updateStep)
			addTime := time.Duration(int64(float64(updateStep)*timeStep*math.Pow10(6))) * time.Microsecond
			timeMapMin = timeMapMin.Add(addTime)
			log.Print(timeMapMin)
			log.Printf("elaps %fsecond update robot cost map %dtimestep, %f added", elap, updateStep, addTime.Seconds())

			// save route file
			csvName := fmt.Sprintf("log/route/%s/%s_%d.csv", now.Format("2006-01-02"), now.Format("01-02-15-4"), rcd.RobotId)
			go grid.SaveRouteCsv(csvName, route, stops)
			//go grid.SaveCostMap(grid.TRWCopy(timeRobotMap))

			if rob, ok := robotList[int(rcd.RobotId)]; ok {
				rob.SetPath(routei)
			} else {
				robotList[int(rcd.RobotId)] = robot.NewRobot(int(rcd.RobotId), robotRadius)
				robotList[int(rcd.RobotId)].SetPath(routei)
			}
		}

	} else if mode == ASTAR3D {
		if gridMap == nil {
			log.Print("not receive gridMap yet ...")
			return
		}
		isx, isy := gridMap.Pos2Ind(float64(rcd.Current.X), float64(rcd.Current.Y))
		igx, igy := gridMap.Pos2Ind(float64(rcd.Destination.X), float64(rcd.Destination.Y))

		routei, err := gridMap.Plan(isx, isy, igx, igy, grid.TRWCopy(timeRobotMap))
		if err != nil {
			log.Print(err)
		} else {
			route := gridMap.Route2Pos(0, routei)
			jsonPayload, err = msg.MakePathMsg(route)
			if err != nil {
				log.Print(err)
			}
			sendPath(jsonPayload, int(rcd.RobotId))
		}
	} else if mode == ASTAR2D {
		route, err := astarPlanner.Plan(float64(rcd.Current.X), float64(rcd.Current.Y), float64(rcd.Destination.X), float64(rcd.Destination.Y))
		if err != nil {
			log.Print(err)
		} else {
			jsonPayload, err = msg.MakePathMsg2D(route)
			if err != nil {
				log.Print(err)
			}
			sendPath(jsonPayload, int(rcd.RobotId))
		}
	}

}

func sendPath(jsonPayload []byte, id int) {
	topic := fmt.Sprintf("robot/path/%d", id)
	mqttProt := proto_mqtt.MQTTRecord{
		Topic:  topic,
		Record: jsonPayload,
	}
	out, err := proto.Marshal(&mqttProt)
	if err != nil {
		log.Print(err)
	}
	cout := api.Content{Entity: out}
	smo := sxutil.SupplyOpts{
		Name:  "robotRoute",
		Cdata: &cout,
	}
	_, err = synerex.MqttClient.NotifySupply(&smo)
	if err != nil {
		log.Print(err)
	} else {
		log.Printf("send path robot%d topic:%s", id, topic)
	}

}

func routeCallback(client *sxutil.SXServiceClient, sp *api.Supply) {
	rcd := &cav.DestinationRequest{}
	err := proto.Unmarshal(sp.Cdata.Entity, rcd)
	if err != nil {
		log.Print(err)
	}
	log.Printf("receive dest request robot%d", rcd.RobotId)
	routeCh <- rcd
	//go routing(rcd)
}

func subsclibeRouteSupply(client *sxutil.SXServiceClient) {
	ctx := context.Background()
	for {
		client.SubscribeSupply(ctx, routeCallback)
		synerex.ReconnectClient(client)
	}
}

func mqttCallback(client *sxutil.SXServiceClient, sp *api.Supply) {
	// ignore my message
	if sp.SenderId == uint64(client.ClientID) {
		return
	}

	rcd := &proto_mqtt.MQTTRecord{}
	err := proto.Unmarshal(sp.Cdata.Entity, rcd)
	if err != nil {
		log.Print("mqtt unmarshal error: ", err)
	} else {
		if strings.HasPrefix(rcd.Topic, "robot/position/") {
			var id int
			fmt.Scanf(rcd.Topic, "robot/position/%d", &id)
			if val, ok := robotList[id]; ok {
				val.SetPos(rcd.Record)
			} else {
				robotList[id] = robot.NewRobot(id, robotRadius)
				robotList[id].SetPos(rcd.Record)
			}
		}
	}

}

func subsclibeMqttSupply(client *sxutil.SXServiceClient) {
	ctx := context.Background()
	for {
		client.SubscribeSupply(ctx, mqttCallback)
		synerex.ReconnectClient(client)
	}
}

func LoggingSettings(logFile string) {
	dir := fmt.Sprintf("log/%s", time.Now().Format("2006-01-02"))
	if _, err := os.Stat(dir); os.IsNotExist(err) {
		os.Mkdir(dir, 0777)
	}
	dir2 := fmt.Sprintf("log/route/%s", time.Now().Format("2006-01-02"))
	if _, err := os.Stat(dir2); os.IsNotExist(err) {
		os.Mkdir(dir2, 0777)
	}
	dir3 := fmt.Sprintf("log/costmap/%s", time.Now().Format("2006-01-02"))
	if _, err := os.Stat(dir3); os.IsNotExist(err) {
		os.Mkdir(dir3, 0777)
	}

	logfile, _ := os.OpenFile(logFile, os.O_RDWR|os.O_CREATE|os.O_APPEND, 0666)
	multiLogFile := io.MultiWriter(os.Stdout, logfile)
	log.SetFlags(log.Ldate | log.Ltime)
	log.SetOutput(multiLogFile)
}

func SetupStaticMap() {
	mapMeta, err := grid.ReadStaticMapImage(yamlFile, mapFile, closeThresh)
	if err != nil {
		log.Print("read map file error: ", err)
	}
	objMap := mapMeta.GetObjectMap()
	reso := *resolution
	if mode == ASTAR2D {
		astarPlanner = astar.NewAstar(objMap, robotRadius, reso)
		log.Print("load astar obj map")
	} else if mode == ASTAR3D {
		gridMap = grid.NewGridMapReso(*mapMeta, robotRadius, reso, objMap)
		timeRobotMap = grid.NewTRW(gridMap.MaxT, gridMap.Width, gridMap.Height)
	} else if mode == ASTAR3DHEXA {
		gridMap = grid.NewGridMapResoHexa(*mapMeta, robotRadius, reso, objMap)
		timeRobotMap = grid.NewTRW(gridMap.MaxT, gridMap.Width, gridMap.Height)
		timeMapMin = time.Now()
	}
}

func main() {
	log.Printf("start geo-routing server mode:%s, timestep:%f, resolution:%f, robotRadius:%f,robotVel: %f/%f, aroundCell: %d, mapfile:%s", mode.String(), timeStep, reso, *robotsize, *robotVel, *robotRotVel, aroundCell, mapFile)
	go sxutil.HandleSigInt()
	wg := sync.WaitGroup{}
	sxutil.RegisterDeferFunction(sxutil.UnRegisterNode)

	//logging configuration
	now := time.Now()
	LoggingSettings("log/" + now.Format("2006-01-02") + "/" + now.Format("2006-01-02-15") + ".log")

	// Synerex Configuration
	synerex.SetupSynerex()

	// load static map data
	SetupStaticMap()

	//start main function
	log.Print("start subscribing")

	go handleRouting()

	//testPath()
	go subsclibeRouteSupply(synerex.RouteClient)
	go subsclibeMqttSupply(synerex.MqttClient)

	//go updateTimeObjMapHandler()

	wg.Add(1)
	wg.Wait()
}
