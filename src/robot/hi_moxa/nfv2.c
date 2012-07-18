#include "nfv2.h"

extern NF_STRUCT_ComBuf 	NFComBuf;

volatile	uint8_t *dataPt;
uint16_t * dummy;
/*
* Funkcja do wrzucenia w obsluge przerwania.
* rxBuf[] - tablica, w ktorej w przerwaniu znak po znaku zapisywana jest odebrana wiadomosc,
* rxPt - indeks pola w tablicy, na ktorym zostala zapisany zostal ostatnio odebrany bajt.
* Funkcja zwraca 1, jesli zostala poprawnie odebrana ramka adresowana do naszego modulu, 0 w pozostalych przypadkach.
*/
uint8_t NF_Interpreter(	volatile uint8_t *rxBuf,
						volatile uint8_t *rxPt,
						volatile uint8_t *commandArray,
						volatile uint8_t *commandCnt){
	static uint8_t n;	
	uint8_t rxAddress, rxParamsCnt;
	uint8_t rxBufIter, combufDataIter, rxDataIter;
	uint8_t *paramsPt;

	if((*rxPt) == 0 && rxBuf[0]!='#'){
		return 0;
	}
	else if((*rxPt) == 3){
		if(( (rxBuf[1] & 0xff) == ((~rxBuf[2]) & 0xff) ) && (rxBuf[1] > 3)){
			n = rxBuf[1];
			(*rxPt)++;
		}
		else{
			(*rxPt) = 0;
		}
		return 0;
	}
	else if((*rxPt) > 3 && (*rxPt) >= n){
		(*rxPt) = 0;
		// if CRC Fail
		if(crcFast(((const uint8_t*)rxBuf) + 3,  n-3) != rxBuf[n]){
			return 0;
		}
		// if Address Fail && not a Broadcast
		//else if(rxBuf[3] != NFComBuf.myAddress && rxBuf[3] != NF_BroadcastAddress){
		//	return 0;
		//}
		// if CRC OK && Address OK go to Command Interpreter
	}
	else{
		(*rxPt)++;
		return 0;
	}
	
	// ############	NF Command Interpreter
	rxAddress = rxBuf[3];
	rxBufIter = 4; // First command starts here
	*commandCnt = 0;
	while(rxBufIter < n){
		paramsPt = (uint8_t*)rxBuf + rxBufIter+2;
		
		// ############### "WRITE" Type Commands
		// if  rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress
		//		Master sends data to me (slave)
		
		// ########	Drives
		// ####		Set Mode
		#ifdef NF_BUFSZ_SetDrivesMode
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesMode){
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesMode;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesMode)) {
						NFComBuf.SetDrivesMode.data[combufDataIter] = ((NF_STRUCT_SetDrivesMode*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetDrivesMode.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Speed
		#ifdef NF_BUFSZ_SetDrivesSpeed
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesSpeed){ 
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesSpeed;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesSpeed)) {
						NFComBuf.SetDrivesSpeed.data[combufDataIter] = ((NF_STRUCT_SetDrivesSpeed*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetDrivesSpeed.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Current
		#ifdef NF_BUFSZ_SetDrivesCurrent
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesCurrent){				
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesCurrent;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesCurrent)) {
						NFComBuf.SetDrivesCurrent.data[combufDataIter] = ((NF_STRUCT_SetDrivesCurrent*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetDrivesCurrent.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Position
		#ifdef NF_BUFSZ_SetDrivesPosition
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesPosition){			
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesPosition;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesPosition)) {
						NFComBuf.SetDrivesPosition.data[combufDataIter] = ((NF_STRUCT_SetDrivesPosition*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetDrivesPosition.updated = 1;
				}
			}
			else
		#endif
		// ####		Set PWM
		#ifdef NF_BUFSZ_SetDrivesPWM
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesPWM){			
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesPWM;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesPWM)) {
						NFComBuf.SetDrivesPWM.data[combufDataIter] = ((NF_STRUCT_SetDrivesPWM*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetDrivesPWM.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Max Current
		#ifdef NF_BUFSZ_SetDrivesMaxCurrent
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesMaxCurrent){				
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesMaxCurrent;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesMaxCurrent)) {
						NFComBuf.SetDrivesMaxCurrent.data[combufDataIter] = ((NF_STRUCT_SetDrivesMaxCurrent*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetDrivesMaxCurrent.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Misc
		#ifdef NF_BUFSZ_SetDrivesMisc
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDrivesMisc){				
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDrivesMisc;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDrivesMisc)) {
						NFComBuf.SetDrivesMisc.data[combufDataIter] = ((NF_STRUCT_SetDrivesMisc*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetDrivesMisc.updated = 1;
				}
			}
			else
		#endif
		
		// ########	Servos
		// ####		Set Mode
		#ifdef NF_BUFSZ_SetServosMode
			if(rxBuf[rxBufIter] == NF_COMMAND_SetServosMode){		
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetServosMode;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetServosMode)) {
						NFComBuf.SetServosMode.data[combufDataIter] = ((NF_STRUCT_SetServosMode*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetServosMode.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Position
		#ifdef NF_BUFSZ_SetServosPosition
			if(rxBuf[rxBufIter] == NF_COMMAND_SetServosPosition){		
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetServosPosition;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetServosPosition)) {
						NFComBuf.SetServosPosition.data[combufDataIter] = ((NF_STRUCT_SetServosPosition*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetServosPosition.updated = 1;
				}
			}
			else
		#endif
		// ####		Set Speed
		#ifdef NF_BUFSZ_SetServosSpeed
			if(rxBuf[rxBufIter] == NF_COMMAND_SetServosSpeed){		
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetServosSpeed;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetServosSpeed)) {
						NFComBuf.SetServosSpeed.data[combufDataIter] = ((NF_STRUCT_SetServosSpeed*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetServosSpeed.updated = 1;
				}
			}
			else
		#endif
		
		// ########	Digital IO
		// ####		Set Outputs
		#ifdef NF_BUFSZ_SetDigitalOutputs
			if(rxBuf[rxBufIter] == NF_COMMAND_SetDigitalOutputs){		
				if(rxAddress == NFComBuf.myAddress || rxAddress == NF_BroadcastAddress) {
					combufDataIter = 0;
					rxDataIter = 0;
					rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_SetDigitalOutputs;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_SetDigitalOutputs)) {
						NFComBuf.SetDigitalOutputs.data[combufDataIter] = ((NF_STRUCT_SetDigitalOutputs*)paramsPt)->data[rxDataIter];
						combufDataIter++;						
						rxDataIter++;
					}
					NFComBuf.SetDigitalOutputs.updated = 1;
				}
			}
			else
		#endif
		
		// ############### "READ" Type Commands
		// if rxParamsCnt == 0 && rxAddress == NFComBuf.myAddress
		//		Master wants to acquire data from me (slave)
		// if rxParamsCnt > 0  && rxAddress == NFComBuf.xxx.addr[i]
		//		Slave returns data that I (master) asked for

		// ########	Drives
		// ####		Read Current
		#ifdef NF_BUFSZ_ReadDrivesCurrent
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDrivesCurrent){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDrivesCurrent;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf.myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDrivesCurrent;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDrivesCurrent)) {
						if(NFComBuf.ReadDrivesCurrent.addr[combufDataIter] == rxAddress) {
							NFComBuf.ReadDrivesCurrent.data[combufDataIter] = ((NF_STRUCT_ReadDrivesCurrent*)paramsPt)->data[rxDataIter];
							rxDataIter++;
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		// ####		Read Position
		#ifdef NF_BUFSZ_ReadDrivesPosition
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDrivesPosition){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDrivesPosition;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf.myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDrivesPosition;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDrivesPosition)) {
						if(NFComBuf.ReadDrivesPosition.addr[combufDataIter] == rxAddress) {
							NFComBuf.ReadDrivesPosition.data[combufDataIter] = ((NF_STRUCT_ReadDrivesPosition*)paramsPt)->data[rxDataIter];
							rxDataIter++;
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		// ####		Read Status
		#ifdef NF_BUFSZ_ReadDrivesStatus
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDrivesStatus){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDrivesStatus;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf.myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDrivesStatus;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDrivesStatus)) {
						if(NFComBuf.ReadDrivesStatus.addr[combufDataIter] == rxAddress) {
							NFComBuf.ReadDrivesStatus.data[combufDataIter] = ((NF_STRUCT_ReadDrivesStatus*)paramsPt)->data[rxDataIter];
							rxDataIter++;
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		
		// ########	Digital IO
		// ####		Read Inputs
		#ifdef NF_BUFSZ_ReadDigitalInputs
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDigitalInputs){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDigitalInputs;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf.myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDigitalInputs;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDigitalInputs)) {
						if(NFComBuf.ReadDigitalInputs.addr[combufDataIter] == rxAddress) {
							NFComBuf.ReadDigitalInputs.data[combufDataIter] = ((NF_STRUCT_ReadDigitalInputs*)paramsPt)->data[rxDataIter];
							rxDataIter++;						
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		
		// ########	Analog IO
		// ####		Read Inputs
		#ifdef NF_BUFSZ_ReadAnalogInputs
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadAnalogInputs){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadAnalogInputs;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf.myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadAnalogInputs;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadAnalogInputs)) {
						if(NFComBuf.ReadAnalogInputs.addr[combufDataIter] == rxAddress) {
							NFComBuf.ReadAnalogInputs.data[combufDataIter] = ((NF_STRUCT_ReadAnalogInputs*)paramsPt)->data[rxDataIter];
							rxDataIter++;						
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		
		// ########	Device
		// ####		Read Status
		#ifdef NF_BUFSZ_ReadDeviceStatus
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDeviceStatus){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDeviceStatus;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf.myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDeviceStatus;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDeviceStatus)) {
						if(NFComBuf.ReadDeviceStatus.addr[combufDataIter] == rxAddress) {
							NFComBuf.ReadDeviceStatus.data[combufDataIter] = ((NF_STRUCT_ReadDeviceStatus*)paramsPt)->data[rxDataIter];
							rxDataIter++;						
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
		// ####		Read Vitals
		#ifdef NF_BUFSZ_ReadDeviceVitals
			if(rxBuf[rxBufIter] == NF_COMMAND_ReadDeviceVitals){
				rxParamsCnt = rxBuf[rxBufIter+1] / NF_DATABYTES_ReadDeviceVitals;
				// Master wants to acquire data from me (slave)
				if((rxParamsCnt == 0) && (rxAddress == NFComBuf.myAddress)){
					commandArray[*commandCnt] = NF_COMMAND_ReadDeviceVitals;
					(*commandCnt) ++;
				}
				// Slave returns data that I (master) asked for
				else if(rxParamsCnt > 0){
					combufDataIter = 0;
					rxDataIter = 0;
					while((rxDataIter < rxParamsCnt) && (combufDataIter < NF_BUFSZ_ReadDeviceVitals)) {
						if(NFComBuf.ReadDeviceVitals.addr[combufDataIter] == rxAddress) {
							NFComBuf.ReadDeviceVitals.data[combufDataIter] = ((NF_STRUCT_ReadDeviceVitals*)paramsPt)->data[rxDataIter];
							rxDataIter++;						
						}
						combufDataIter++;
					}
				}
			}
			else
		#endif
				NFComBuf.unknownCommandRec = 1;
			
		
		rxBufIter += rxBuf[rxBufIter+1]+2;
	}
	
	return rxBufIter;
}

uint8_t NF_MakeCommandFrame(uint8_t *txBuf, const uint8_t *commandArray, uint8_t commandCnt, uint8_t txAddress){
	uint8_t txBufIter, commandIter, txDataIter, combufDataIter;	

	 
	txBuf[0] = '#';
	txBuf[3] = txAddress;

	txBufIter=4;
	commandIter=0;
		
	while(commandIter < commandCnt){
	
		dataPt = txBuf+txBufIter+2;

		// ########	Drives
		// ####		Set Mode
		#ifdef NF_BUFSZ_SetDrivesMode
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesMode){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesMode) {
					if(NFComBuf.SetDrivesMode.addr[combufDataIter] == txAddress) {
						((NF_STRUCT_SetDrivesMode*)dataPt)->data[txDataIter] = NFComBuf.SetDrivesMode.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesMode;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesMode;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Speed
		#ifdef NF_BUFSZ_SetDrivesSpeed
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesSpeed){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesSpeed) {
					if(NFComBuf.SetDrivesSpeed.addr[combufDataIter] == txAddress) {
						((NF_STRUCT_SetDrivesSpeed*)dataPt)->data[txDataIter] = NFComBuf.SetDrivesSpeed.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesSpeed;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesSpeed;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Current
		#ifdef NF_BUFSZ_SetDrivesCurrent
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesCurrent){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesCurrent) {
					if(NFComBuf.SetDrivesCurrent.addr[combufDataIter] == txAddress) {
						((NF_STRUCT_SetDrivesCurrent*)dataPt)->data[txDataIter] = NFComBuf.SetDrivesCurrent.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesCurrent;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesCurrent;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Position
		#ifdef NF_BUFSZ_SetDrivesPosition
			if(commandArray[commandIter] == NF_COMMAND_SetDrivesPosition){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDrivesPosition) {
					if(NFComBuf.SetDrivesPosition.addr[combufDataIter] == txAddress) {
						((NF_STRUCT_SetDrivesPosition*)dataPt)->data[txDataIter] = NFComBuf.SetDrivesPosition.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDrivesPosition;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDrivesPosition;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif	

		// ########	Servos
		// ####		Set Mode
		#ifdef NF_BUFSZ_SetServosMode
			if(commandArray[commandIter] == NF_COMMAND_SetServosMode){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetServosMode) {
					if(NFComBuf.SetServosMode.addr[combufDataIter] == txAddress) {
						((NF_STRUCT_SetServosMode*)dataPt)->data[txDataIter] = NFComBuf.SetServosMode.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetServosModen;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetServosMode;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Position
		#ifdef NF_BUFSZ_SetServosPosition
			if(commandArray[commandIter] == NF_COMMAND_SetServosPosition){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetServosPosition) {
					if(NFComBuf.SetServosPosition.addr[combufDataIter] == txAddress) {
						((NF_STRUCT_SetServosPosition*)dataPt)->data[txDataIter] = NFComBuf.SetServosPosition.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetServosPosition;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetServosPosition;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif
		// ####		Set Speed
		#ifdef NF_BUFSZ_SetServosSpeed
			if(commandArray[commandIter] == NF_COMMAND_SetServosSpeed){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetServosSpeed) {
					if(NFComBuf.SetServosSpeed.addr[combufDataIter] == txAddress) {
						((NF_STRUCT_SetServosSpeed*)dataPt)->data[txDataIter] = NFComBuf.SetServosSpeed.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetServosSpeed;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetServosSpeed;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif	  

		// ########	Digital IO
		// ####		Set Outputs
		#ifdef NF_BUFSZ_SetDigitalOutputs
			if(commandArray[commandIter] == NF_COMMAND_SetDigitalOutputs){
				combufDataIter = 0;
				txDataIter = 0;
				while(combufDataIter < NF_BUFSZ_SetDigitalOutputs) {
					if(NFComBuf.SetDigitalOutputs.addr[combufDataIter] == txAddress) {
						((NF_STRUCT_SetDigitalOutputs*)dataPt)->data[txDataIter] = NFComBuf.SetDigitalOutputs.data[combufDataIter];
						txDataIter++;						
					}
					combufDataIter++;
				}
				if(txDataIter > 0){
					txBuf[txBufIter] = NF_COMMAND_SetDigitalOutputs;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_SetDigitalOutputs;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
			}
			else
		#endif 
		
		// ############### "READ" Type Commands
		// if txAddress == NFComBuf.myAddress
		//		I (slave) return data that master asked for
		// if rxAddress == NFComBuf.xxx.addr[i]
		//		I (master) want to acquire data from slave
		
		// ########	Device
		// ####		Read Status
		#ifdef NF_BUFSZ_ReadDeviceStatus
			if(commandArray[commandIter] == NF_COMMAND_ReadDeviceStatus){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf.myAddress){					
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadDeviceStatus) {
						((NF_STRUCT_ReadDeviceStatus*)dataPt)->data[txDataIter] = NFComBuf.ReadDeviceStatus.data[combufDataIter];
						txDataIter++;						
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadDeviceStatus;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadDeviceStatus;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadDeviceStatus;
					txBuf[txBufIter+1] = 0;	
					txBufIter += 2;
					
				}
			}
			else
		#endif
		// ####		Read Vitals
		#ifdef NF_BUFSZ_ReadDeviceVitals
			if(commandArray[commandIter] == NF_COMMAND_ReadDeviceVitals){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf.myAddress){					
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadDeviceVitals) {
						((NF_STRUCT_ReadDeviceVitals*)dataPt)->data[txDataIter] = NFComBuf.ReadDeviceVitals.data[combufDataIter];
						txDataIter++;						
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadDeviceVitals;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadDeviceVitals;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadDeviceVitals;
					txBuf[txBufIter+1] = 0;	
					txBufIter += 2;
					
				}
			}
			else
		#endif
		
		// ########	Drives
		// ####		Read Status
		#ifdef NF_BUFSZ_ReadDrivesStatus
			if(commandArray[commandIter] == NF_COMMAND_ReadDrivesStatus){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf.myAddress){					
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadDrivesStatus) {
						//*dataPt = NFComBuf.ReadDrivesStatus.data[combufDataIter];
												
						dummy = &(((NF_STRUCT_ReadDrivesStatus*)dataPt)->data[0]);
						*dummy = NFComBuf.ReadDrivesStatus.data[combufDataIter];
						//((NF_STRUCT_ReadDrivesStatus*)dataPt)->data[txDataIter] = NFComBuf.ReadDrivesStatus.data[combufDataIter];
						txDataIter++;						
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadDrivesStatus;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadDrivesStatus;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadDrivesStatus;
					txBuf[txBufIter+1] = 0;	
					txBufIter += 2;
					
				}
			}
			else
		#endif

		// ########	Analog IO
		// ####		Read Inputs
		#ifdef NF_BUFSZ_ReadAnalogInputs
			if(commandArray[commandIter] == NF_COMMAND_ReadAnalogInputs){
				// I (slave) return data that master asked for
				if(txAddress == NFComBuf.myAddress){					
					combufDataIter = 0;
					txDataIter = 0;
					while(combufDataIter < NF_BUFSZ_ReadAnalogInputs) {
						((NF_STRUCT_ReadAnalogInputs*)dataPt)->data[txDataIter] = NFComBuf.ReadAnalogInputs.data[combufDataIter];
						txDataIter++;						
						combufDataIter++;
					}
					txBuf[txBufIter] = NF_COMMAND_ReadAnalogInputs;
					txBuf[txBufIter+1] = txDataIter * NF_DATABYTES_ReadAnalogInputs;	
					txBufIter += txBuf[txBufIter+1]+2;			
				}
				// I (master) want to acquire data from slave
				else{
					txBuf[txBufIter] = NF_COMMAND_ReadAnalogInputs;
					txBuf[txBufIter+1] = 0;	
					txBufIter += 2;
					
				}
			}
			else
		#endif
		
			{
				NFComBuf.unknownCommandSend = 1;  
				commandIter++;
				continue;
			}
		commandIter++;
	}
	txBuf[txBufIter] = crcFast(((const uint8_t*)txBuf) + 3,  txBufIter-3);
	txBuf[1] = txBufIter;
	txBuf[2] = ~txBufIter;


	return txBufIter+1;
}
