package routing

import (
	"encoding/json"
	"errors"
	"fmt"
	"io/ioutil"
	"log"
	"math"
	"time"
)

// initialize custom resolution
// using main
func NewGridMapResoHexa(m MapMeta, robotRadius float64, resolution float64, objMap [][2]float64) *GridMap {
	start := time.Now()
	g := new(GridMap)
	g.MapOrigin = m.Origin
	g.Resolution = resolution

	var aList []float64
	var bList []float64

	for _, obj := range objMap {
		aList = append(aList, getA(obj[0], obj[1]))
		bList = append(bList, getB(obj[0], obj[1]))
	}

	maxX := MaxFloat(aList)
	maxY := MaxFloat(bList)
	g.Origin.X = MinFloat(aList)
	g.Origin.Y = MinFloat(bList)

	g.Width = int(math.Round((maxX - g.Origin.X) / resolution))
	g.Height = int(math.Round((maxY - g.Origin.Y) / resolution))

	g.MaxT = MaxTimeLength
	g.ObjectMap = make(map[Index]bool, g.Height)

	count := 0
	for j := 0; j < g.Height; j++ {
		b := g.Origin.Y + float64(j)*g.Resolution
		for i := 0; i < g.Width; i++ {
			a := g.Origin.X + float64(i)*g.Resolution
			x := getXAB(a, b)
			y := getYAB(a, b)
			g.ObjectMap[newIndex(i, j)] = false
			for _, op := range objMap {
				d := math.Hypot(op[0]-x, op[1]-y)
				if d <= robotRadius {
					g.ObjectMap[newIndex(i, j)] = true
					count += 1
					break
				}
			}
		}
	}

	elaps := time.Since(start).Seconds()
	log.Printf("loading gridmap resolution: %f, takes: %f seconds, obj %d counts, width: %d, height: %d, origin %f,%f", resolution, elaps, count, g.Width, g.Height, g.Origin.X, g.Origin.Y)
	return g
}

func getA(x, y float64) float64 {
	return x/math.Sqrt(3) + y
}

func getB(x, y float64) float64 {
	return x/math.Sqrt(3) - y
}

func A(x, y float64) float64 {
	return x/math.Sqrt(3) + y
}

func B(x, y float64) float64 {
	return x/math.Sqrt(3) - y
}

func getXAB(a, b float64) float64 {
	return math.Sqrt(3)/2*a + math.Sqrt(3)/2*b
}

func getYAB(a, b float64) float64 {
	return a/2 - b/2
}

func (g GridMap) Pos2IndHexa(x, y float64) (int, int) {
	if x < g.MapOrigin.X || y < g.MapOrigin.Y {
		log.Printf("position (%f,%f) is out of map", x, y)
		return 0, 0
	}
	a := getA(x, y)
	b := getB(x, y)
	xid := int(math.Round((a - g.Origin.X) / g.Resolution))
	yid := int(math.Round((b - g.Origin.Y) / g.Resolution))
	return xid, yid
}

type logOpt struct {
	X         int
	Y         int
	T         int
	Cost      float64
	StopCount int
}

func (m GridMap) PlanHexa(id int, sa, sb, ga, gb int, v, w, timeStep float64, TRW TimeRobotMap, otherRobot map[Index]bool) (route [][3]int, oerr error) {
	startTime := time.Now()
	var logData []logOpt

	sx := int(getXAB(float64(sa), float64(sb)))
	sy := int(getYAB(float64(sa), float64(sb)))
	gx := int(getXAB(float64(ga), float64(gb)))
	gy := int(getYAB(float64(ga), float64(gb)))

	//timeStep := m.Resolution/v + 2*math.Pi/3/w // L/v + 2pi/3w  120度回転したときの一番かかる時間
	log.Printf("start planning robot%d (%f, %f) to (%f, %f)",
		id,
		m.MapOrigin.X+float64(sx)*m.Resolution,
		m.MapOrigin.Y+float64(sy)*m.Resolution,
		m.MapOrigin.X+float64(gx)*m.Resolution,
		m.MapOrigin.Y+float64(gy)*m.Resolution,
	)

	if m.ObjectMap[newIndex(ga, gb)] {
		oerr = fmt.Errorf("robot%d path planning error: goal is not verified", id)
		return nil, oerr
	}
	if m.ObjectMap[newIndex(sa, sb)] {
		oerr = fmt.Errorf("robot%d path planning error: start point is not verified", id)
		return nil, oerr
	}
	start := &Node{T: 0, XId: sa, YId: sb, Cost: 0, Parent: nil}
	goal := &Node{T: 0, XId: ga, YId: gb, Cost: 0, Parent: nil}

	openSetT := make(map[IndexT]*Node)
	openSet := make(map[Index]*Node)

	closeSet := make(map[Index]*Node)
	closeSetT := make(map[IndexT]*Node)

	openSetT[nodeIndexT(start)] = start
	openSet[nodeIndex(start)] = start

	count := 0
	current := &Node{T: 0}
	var minTime int

	for {
		count += 1
		// failure
		if len(openSetT) == 0 {
			elaps := time.Since(startTime).Seconds()
			log.Print(current.T, current.XId, current.YId, count, elaps)
			oerr = errors.New("path planning error: open set is empty")
			bytes, _ := json.Marshal(logData)
			ioutil.WriteFile(fmt.Sprintf("log/route/fail_route%d_%s.log", id, time.Now().Format("01-02-15-4")), bytes, 0666)
			return nil, oerr
		}

		// 20秒以上で諦める
		if time.Since(startTime).Seconds() > 20 {
			log.Printf("path planning time out. count %d current is (%d, %d, %d)", count, current.T, current.XId, current.YId)
			bytes, jerr := json.MarshalIndent(logData, "", " ")
			if jerr != nil {
				log.Print(jerr)
			}
			ioutil.WriteFile(fmt.Sprintf("log/route/fail_route%d_%s.log", id, time.Now().Format("01-02-15-4")), bytes, 0666)
			oerr = errors.New("path planning timeouted")
			return m.finalPath(goal, closeSetT), oerr
		}

		// get minimum cost node in open set
		minCost := 9999999999999999999.9
		minTime = 99999999999999999
		var minKey IndexT
		for key, val := range openSetT {
			calCost := val.Cost + heuristicHexa(goal, val)/v // length to goal / vel
			if calCost < minCost {
				minCost = calCost
				minKey = key
			}
			if val.T < minTime {
				minTime = val.T
			}
		}
		current = openSetT[minKey]
		logData = append(logData, logOpt{current.XId, current.YId, current.T, current.Cost, current.StopCount})

		// find Goal
		if current.XId == ga && current.YId == gb {
			log.Print("find goal")
			bytes, jerr := json.MarshalIndent(logData, "", " ")
			if jerr != nil {
				log.Print(jerr)
			}
			ioutil.WriteFile(fmt.Sprintf("log/route/route%d_%s.log", id, time.Now().Format("01-02-15-4")), bytes, 0666)
			goal.Parent = current.Parent
			goal.Cost = current.Cost
			goal.T = current.T
			elaps := time.Since(startTime).Seconds()
			route = m.finalPath(goal, closeSetT)
			log.Printf("robot%d planning (%f, %f) to (%f, %f) took %f seconds, count: %d, length: %d",
				id,
				m.MapOrigin.X+float64(sx)*m.Resolution,
				m.MapOrigin.Y+float64(sy)*m.Resolution,
				m.MapOrigin.X+float64(gx)*m.Resolution,
				m.MapOrigin.Y+float64(gy)*m.Resolution,
				elaps,
				count,
				len(route),
			)
			return route, nil
		}

		delete(openSetT, minKey)
		closeSet[nodeIndex(current)] = current
		closeSetT[nodeIndexT(current)] = current

		around := current.AroundHexa(&m, minTime, v, w, timeStep, TRW, otherRobot)
		for _, an := range around {
			indT := nodeIndexT(an)
			ind := nodeIndex(an)

			// すでに通った道を戻るのはだめ
			if _, ok := closeSet[ind]; ok {
				// ただし、止まる場合を除く
				if an.Parent.XId != an.XId || an.Parent.YId != an.YId {
					continue
				}
			}

			//二次元上で同じ座標がなければ入れる
			if _, ok := openSet[ind]; !ok {
				openSet[ind] = an
				if _, ok := openSetT[indT]; !ok {
					openSetT[indT] = an
				}
				//二回目以降の二次元座表のときは外す
			} else {
				// ただし、止まる場合を除く
				if an.Parent.XId == an.XId || an.Parent.YId == an.YId {
					if _, ok := openSetT[indT]; !ok {
						openSetT[indT] = an
					}
				}
			}

		}
	}
}

func (g GridMap) Route2PosHexa(minT float64, timeStep float64, route [][3]int) [][3]float64 {
	l := len(route)
	fRoute := make([][3]float64, l)

	for i, r := range route {
		a, b := g.Ind2Pos(r[1], r[2])
		x := getXAB(a, b)
		y := getYAB(a, b)
		t := minT + float64(r[0])*timeStep
		p := [3]float64{t, x, y}
		fRoute[i] = p
	}
	return fRoute
}

func (n Node) AroundHexa(g *GridMap, minTime int, v, w, timeStep float64, TRW TimeRobotMap, otherRobot map[Index]bool) []*Node {

	// var diffA int
	// var diffB int
	// if n.Parent != nil {
	// 	diffA = n.XId - n.Parent.XId
	// 	diffB = n.YId - n.Parent.YId
	// }

	cost1 := g.Resolution / v
	// [time, x, y, cost]
	motion := [7][4]float64{
		{1.0, 0.0, 0.0, timeStep},
		{0.0, 1.0, 0.0, cost1},
		{0.0, 0.0, 1.0, cost1},
		{0.0, -1.0, 1.0, cost1},
		{0.0, -1.0, 0.0, cost1},
		{0.0, 0.0, -1.0, cost1},
		{0.0, 1.0, -1.0, cost1},
	}
	var around []*Node
	for i, m := range motion {
		aX := n.XId + int(m[1])
		aY := n.YId + int(m[2])
		aT := n.T + int(m[0]) + int(timeStep)

		//時間コストマップ外の時間は外す
		if aT >= g.MaxT {
			continue
		}

		//map外のノードは外す
		if aX < 0 || aX >= g.Width {
			continue
		}
		if aY < 0 || aY >= g.Height {
			continue
		}

		//元から障害物で通れないところは外す
		if g.ObjectMap[newIndex(aX, aY)] {
			continue
		}

		//他のロボットがいる場所は外す
		if val, ok := otherRobot[newIndex(aX, aY)]; ok {
			if val {
				continue
			}
		}

		//移動中のロボットがいて通れないところは外す
		if val, ok := TRW[newIndexT(aT, aX, aY)]; ok {
			if val {
				continue
			}
		}

		var newCost = n.Cost + m[3]
		// stay してないとき
		// if diffA != 0 && diffB != 0 && i != 0 {
		// 	// 前のグリッドからそのまま直進できるならコスト少なめ
		// 	if m[1] == float64(diffA) && m[2] == float64(diffB) {
		// 		newCost = n.Cost + m[3]

		// 	} else {
		// 		// 回転が必要なら回転分のコストを加える
		// 		newCost = n.Cost + m[3] + math.Pi/w/3
		// 	}
		// }

		node := n.NewNode(aT, aX, aY, newCost)

		// MaxStopCount以上止まりすぎはだめ
		if i == 0 {
			node.StopCount = n.StopCount + 1
			if node.StopCount > MaxStopCount {
				continue
			}
		}

		around = append(around, node)
	}
	return around
}

func heuristicHexa(n1, n2 *Node) float64 {
	w := 1.0
	// x1 := getXAB(float64(n1.XId), float64(n1.YId))
	// y1 := getYAB(float64(n1.XId), float64(n1.YId))
	// x2 := getXAB(float64(n2.XId), float64(n2.YId))
	// y2 := getYAB(float64(n2.XId), float64(n2.YId))
	// d := w * math.Hypot(x1-x2, y1-y2)

	d := w * math.Hypot(float64(n1.XId)-float64(n2.XId), float64(n1.YId)-float64(n2.YId))
	return d
}

func sumHexa(n1, n2 *Node) float64 {
	w := 1.0
	d := w*math.Abs(float64(n1.XId)-float64(n2.XId)) + math.Abs(float64(n1.YId)-float64(n2.YId))
	return d
}
