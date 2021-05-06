package routing

func (g GridMap) UpdateStep(TW TimeRobotMap, step int) {
	newTRW := make(map[IndexT]bool)
	for key, val := range TW {
		// t := int(key / IndexT(HASH2))
		// x := int(key % IndexT(HASH2))
		// y := int((key / IndexT(HASH)) % IndexT(HASH))
		t, x, y := key.GetXYT()
		if t < MaxTimeLength-step-1 {
			newTRW[newIndexT(t+step, x, y)] = val
		}
	}
}

func (key IndexT) GetXYT() (int, int, int) {
	t := int(key / IndexT(HASH2))
	x := int(key % IndexT(HASH2))
	y := int((key / IndexT(HASH)) % IndexT(HASH))
	return t, x, y
}

func TRWCopy(current TimeRobotMap) TimeRobotMap {
	trw := make(map[IndexT]bool)

	for key, val := range current {
		trw[key] = val
	}
	return trw
}

func (g GridMap) UpdateTimeObjMapHexa(TW TimeRobotMap, route [][3]int, robotRadius float64, timeStep int) {
	around := [6][2]int{{-1, 0}, {0, -1}, {1, -1}, {1, 0}, {0, 1}, {-1, 1}}
	aroundMore := [6][2]int{{2, -1}, {1, -2}, {-1, -1}, {-2, 1}, {2, -1}, {1, 1}}

	var it int
	var ix int
	var iy int
	for i := 0; i < len(route); i++ {
		for k := 0; k < timeStep; k++ {
			it = route[i][0] + k
			ix = route[i][1]
			iy = route[i][2]
			TW[newIndexT(it, ix, iy)] = true
			if robotRadius < g.Resolution {
				continue
			} else if robotRadius <= 2*g.Resolution {
				for _, v := range around {
					ny := iy + v[1]
					nx := ix + v[0]
					if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
						continue
					}
					TW[newIndexT(it, nx, ny)] = true
				}
			} else { //周囲18マス
				for _, v := range around {
					for d := 1; d <= 2; d++ {
						ny := iy + v[1]*d
						nx := ix + v[0]*d
						if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
							continue
						}
						TW[newIndexT(it, nx, ny)] = true
					}
				}
				for _, v := range aroundMore {
					ny := iy + v[1]
					nx := ix + v[0]
					if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
						continue
					}
					TW[newIndexT(it, nx, ny)] = true
				}
			}
		}
	}
	for j := it; j < MaxTimeLength; j++ {
		TW[newIndexT(it+j, ix, iy)] = true
	}
}

// func (g GridMap) UpdateTimeRobotMap(route [][3]int, robotRadius float64, TRW TimeRobotMap) {
// 	around8 := [8][2]int{{-1, 0}, {0, 1}, {1, 0}, {0, -1}, {-1, -1}, {-1, 1}, {1, 1}, {1, -1}}
// 	around4 := [4][2]int{{-1, 0}, {0, 1}, {1, 0}, {0, -1}}

// 	for i := 0; i < len(route); i++ {
// 		it := route[i][0]
// 		ix := route[i][1]
// 		iy := route[i][2]
// 		TRW[it][iy][ix] = true
// 		if robotRadius < g.Resolution {
// 			continue
// 		} else if robotRadius <= math.Sqrt(2)*g.Resolution {
// 			for _, v := range around4 {
// 				ny := iy + v[1]
// 				nx := ix + v[0]
// 				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
// 					continue
// 				}
// 				TRW[it][ny][nx] = true
// 			}
// 		} else {
// 			for _, v := range around8 {
// 				ny := iy + v[1]
// 				nx := ix + v[0]
// 				if ny < 0 || nx < 0 || nx >= g.Width || ny >= g.Height {
// 					continue
// 				}
// 				TRW[it][ny][nx] = true
// 			}
// 		}

// 	}
// }
