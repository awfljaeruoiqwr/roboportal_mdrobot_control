package bot

import (
	"encoding/json"
	"fmt"
	"log"

	"github.com/pion/mediadevices"
	"github.com/pion/webrtc/v3"
	"github.com/roboportal/bot_box/pkg/botcom"
	"github.com/roboportal/bot_box/pkg/utils"
)

const (
	Idle       = "Idle"
	Connecting = "Connecting"
	Connected  = "Connected"
)

type ABot struct {
	ClosePeerConnectionChan   chan struct{}
	QuitWebRTCChan            chan struct{}
	WebRTCConnectionStateChan chan string
	DescriptionChan           chan webrtc.SessionDescription
	CandidateChan             chan webrtc.ICECandidateInit
	ArenaDescriptionChan      chan webrtc.SessionDescription
	ArenaCandidateChan        chan webrtc.ICECandidateInit
	SendDataChan              chan string
	WSConStatChan             chan string
	ControlsReadyChan         chan bool
	ID                        int
	Status                    string
	IsReady                   bool
	ConnectionID              string
}

func (b *ABot) SetIdle() {
	b.Status = Idle
	b.IsReady = false
}

func (b *ABot) SetConnecting() {
	b.Status = Connecting
	b.IsReady = false
}

func (b *ABot) SetConnected() {
	b.Status = Connected
}

func (b *ABot) ClearConnectionID() {
	b.ConnectionID = ""
}

func (b *ABot) NotifyAreControlsAllowedBySupervisorChange(state bool) {
	if b.Status != Connected {
		return
	}

	status := "DECLINED"
	if state {
		status = "ALLOWED"
	}
	command := fmt.Sprintf("{\"type\": \"CONTROLS_SUPERVISOR_STATUS_CHANGE\", \"payload\": {\"status\": \"%s\"}}", status)

	b.SendDataChan <- command
}

type RunParams struct {
	StunUrls                          []string
	TokenString                       string
	PublicKey                         string
	Api                               *webrtc.API
	MediaStream                       mediadevices.MediaStream
	WsWriteChan                       chan string
	BotCommandsWriteChan              chan string
	GetAreControlsAllowedBySupervisor func() bool
	GetAreBotsReady                   func() bool
	SetBotReady                       func(int)
	SetBotNotReady                    func(int)
	IsAudioOutputEnabled              bool
	CameraSelectChan          				chan string
}

type CreateConnectionPayload struct {
	Token     string `json:"token"`
	PublicKey string `json:"publicKey"`
	ID        int    `json:"id"`
}

type CreateConnectionAction struct {
	Name    string                  `json:"name"`
	Payload CreateConnectionPayload `json:"payload"`
}

func buildCreateConnectionMessage(id int, publicKey string, tokenString string) CreateConnectionAction {
	return CreateConnectionAction{
		Name: "CREATE_CONNECTION",
		Payload: CreateConnectionPayload{
			Token:     tokenString,
			PublicKey: publicKey,
			ID:        id,
		},
	}
}

func (b *ABot) Run(p RunParams) {
	log.Println("Creating connection for bot: ", b.ID)
	message := buildCreateConnectionMessage(b.ID, p.PublicKey, p.TokenString)

	br, err := json.Marshal(message)

	if err != nil {
		log.Println("Serialize 'CREATE_CONNECTION' message to RoboPortal error", err)
		return
	}

	go func() {
		log.Println("Sending create connection message for bot: ", b.ID)
		p.WsWriteChan <- string(br)
	}()

	botcomParams := botcom.InitParams{
		Id:                                b.ID,
		StunUrls:                          p.StunUrls,
		Api:                               p.Api,
		MediaStream:                       p.MediaStream,
		DescriptionChan:                   b.DescriptionChan,
		CandidateChan:                     b.CandidateChan,
		ArenaDescriptionChan:              b.ArenaDescriptionChan,
		ArenaCandidateChan:                b.ArenaCandidateChan,
		WebRTCConnectionStateChan:         b.WebRTCConnectionStateChan,
		SendDataChan:                      b.SendDataChan,
		QuitWebRTCChan:                    b.QuitWebRTCChan,
		ClosePeerConnectionChan:           b.ClosePeerConnectionChan,
		BotCommandsWriteChan:              p.BotCommandsWriteChan,
		ControlsReadyChan:                 b.ControlsReadyChan,
		GetAreControlsAllowedBySupervisor: p.GetAreControlsAllowedBySupervisor,
		GetAreBotsReady:                   p.GetAreBotsReady,
		IsAudioOutputEnabled:              p.IsAudioOutputEnabled,
		ClearBotConnectionID:              b.ClearConnectionID,
		CameraSelectChan: 								 p.CameraSelectChan,
	}

	log.Println("Init webrtc communicator for bot: ", b.ID)

	go botcom.Init(botcomParams)

	for {
		select {

		case description := <-b.ArenaDescriptionChan:
			type SetDescriptionPayload struct {
				Token       string                    `json:"token"`
				PublicKey   string                    `json:"publicKey"`
				Description webrtc.SessionDescription `json:"description"`
				ID          int                       `json:"id"`
			}

			type SetOfferAction struct {
				Name    string                `json:"name"`
				Payload SetDescriptionPayload `json:"payload"`
			}

			log.Println("Sending answer for bot: ", b.ID)
			message := SetOfferAction{
				Name: "SET_DESCRIPTION",
				Payload: SetDescriptionPayload{
					Token:       p.TokenString,
					PublicKey:   p.PublicKey,
					Description: description,
					ID:          b.ID,
				},
			}

			b, err := json.Marshal(message)

			if err != nil {
				log.Println("Serialize 'SET_DESCRIPTION' message to RoboPortal error", err)
				return
			}

			p.WsWriteChan <- string(b)

		case candidate := <-b.ArenaCandidateChan:
			type SetCandidatePayload struct {
				Token     string                  `json:"token"`
				PublicKey string                  `json:"publicKey"`
				Candidate webrtc.ICECandidateInit `json:"candidate"`
				ID        int                     `json:"id"`
			}

			type SetCandidateAction struct {
				Name    string              `json:"name"`
				Payload SetCandidatePayload `json:"payload"`
			}

			log.Println("Sending candidate for bot: ", b.ID)
			message := SetCandidateAction{
				Name: "SET_CANDIDATE",
				Payload: SetCandidatePayload{
					Token:     p.TokenString,
					PublicKey: p.PublicKey,
					Candidate: candidate,
					ID:        b.ID,
				},
			}

			b, err := json.Marshal(message)

			if err != nil {
				log.Println("Serialize 'SET_CANDIDATE' message to RoboPortal error", err)
				return
			}

			p.WsWriteChan <- string(b)

		case state := <-b.WebRTCConnectionStateChan:
			if state == webrtc.ICEConnectionStateConnected.String() {
				b.SetConnected()

				type BotConnectedPayload struct {
					Token     string `json:"token"`
					PublicKey string `json:"publicKey"`
					ID        int    `json:"id"`
				}

				type BotConnectedAction struct {
					Name    string              `json:"name"`
					Payload BotConnectedPayload `json:"payload"`
				}

				log.Println("Bot connected via WebRTC: ", b.ID)
				message := BotConnectedAction{
					Name: "BOT_CONNECTED",
					Payload: BotConnectedPayload{
						Token:     p.TokenString,
						PublicKey: p.PublicKey,
						ID:        b.ID,
					},
				}

				command, err := json.Marshal(message)

				if err != nil {
					log.Println("Serialize 'BOT_CONNECTED' message to RoboPortal error", err)
					return
				}

				p.WsWriteChan <- string(command)

			}
			if state == webrtc.ICEConnectionStateFailed.String() ||
				state == webrtc.ICEConnectionStateClosed.String() {
				b.SetIdle()

				type UnblockDisconnectedPayload struct {
					Token     string `json:"token"`
					PublicKey string `json:"publicKey"`
					ID        int    `json:"id"`
				}

				type UnblockDisconnectedAction struct {
					Name    string                     `json:"name"`
					Payload UnblockDisconnectedPayload `json:"payload"`
				}

				log.Println("Unblocking disconnected bot: ", b.ID, state)

				message := UnblockDisconnectedAction{
					Name: "UNBLOCK_DISCONNECTED",
					Payload: UnblockDisconnectedPayload{
						Token:     p.TokenString,
						PublicKey: p.PublicKey,
						ID:        b.ID,
					},
				}

				command, err := json.Marshal(message)

				if err != nil {
					log.Println("Serialize 'UNBLOCK_DISCONNECTED' message to RoboPortal error", err)
					return
				}

				p.WsWriteChan <- string(command)

				go utils.TriggerChannel(b.QuitWebRTCChan)
			}

		case state := <-b.ControlsReadyChan:
			log.Println("Bot ready:", b.ID, state)

			status := "DECLINED"
			if state {
				status = "ALLOWED"
			}
			command := fmt.Sprintf("{\"type\": \"CONTROLS_STATUS_CHANGE\", \"payload\": {\"status\": \"%s\"}}", status)

			b.SendDataChan <- command

			if state {
				p.SetBotReady(b.ID)
			} else {
				p.SetBotNotReady(b.ID)
			}

		case wsConnectionStatus := <-b.WSConStatChan:
			if wsConnectionStatus == "connected" {
				log.Println("RE-creating connection for bot: ", b.ID)

				message := buildCreateConnectionMessage(b.ID, p.PublicKey, p.TokenString)

				br, err := json.Marshal(message)

				if err != nil {
					log.Println("Serialize 'CREATE_CONNECTION' message to RoboPortal error", err)
					return
				}

				p.WsWriteChan <- string(br)
			}
		}
	}
}

func Factory(id int) ABot {
	return ABot{
		QuitWebRTCChan:            make(chan struct{}, 1),
		ClosePeerConnectionChan:   make(chan struct{}, 1),
		WebRTCConnectionStateChan: make(chan string, 10),
		DescriptionChan:           make(chan webrtc.SessionDescription, 10),
		CandidateChan:             make(chan webrtc.ICECandidateInit, 10),
		ArenaDescriptionChan:      make(chan webrtc.SessionDescription, 10),
		ArenaCandidateChan:        make(chan webrtc.ICECandidateInit, 10),
		SendDataChan:              make(chan string, 1000),
		ControlsReadyChan:         make(chan bool, 10),
		WSConStatChan:             make(chan string, 10),
		ID:                        id,
		Status:                    Idle,
		IsReady:                   false,
		ConnectionID:              "",
	}
}
