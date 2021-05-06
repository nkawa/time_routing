package synerex

import (
	"flag"
	"log"
	"sync"
	"time"

	syp "github.com/synerex/synerex_proto"
	sxutil "github.com/synerex/synerex_sxutil"
)

var (
	Nodesrv = flag.String("nodesrv", "127.0.0.1:9990", "node serv address")

	Mu              sync.Mutex
	SxServerAddress string

	//synerex client
	MqttClient  *sxutil.SXServiceClient
	RouteClient *sxutil.SXServiceClient
)

//synerex recconect to client
func ReconnectClient(client *sxutil.SXServiceClient) {
	Mu.Lock()
	if client.SXClient != nil {
		client.SXClient = nil
		log.Printf("Client reset \n")
	}
	Mu.Unlock()
	time.Sleep(5 * time.Second) // wait 5 seconds to reconnect
	Mu.Lock()
	if client.SXClient == nil {
		newClt := sxutil.GrpcConnectServer(SxServerAddress)
		if newClt != nil {
			// log.Printf("Reconnect server [%s]\n", s.SxServerAddress)
			client.SXClient = newClt
		}
	}
	Mu.Unlock()
}

func SetupSynerex() {
	channels := []uint32{syp.MQTT_GATEWAY_SVC, syp.ROUTING_SERVICE}
	srv, err := sxutil.RegisterNode(*Nodesrv, "GeoRoutingProvider", channels, nil)
	if err != nil {
		log.Fatal("can not registar node")
	}
	log.Printf("connectiong server [%s]", srv)
	SxServerAddress = srv

	synerexClient := sxutil.GrpcConnectServer(srv)
	argJson1 := "{Client: GeoMQTT}"
	MqttClient = sxutil.NewSXServiceClient(synerexClient, syp.MQTT_GATEWAY_SVC, argJson1)
	argJson2 := "{Client: GeoRoute}"
	RouteClient = sxutil.NewSXServiceClient(synerexClient, syp.ROUTING_SERVICE, argJson2)
}
