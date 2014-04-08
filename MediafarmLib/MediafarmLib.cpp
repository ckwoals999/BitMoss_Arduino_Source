// This is the main DLL file.

//#include "stdafx.h"

#include "MediafarmLib.h"


MediafarmLib::MediafarmLib() {

}


/*
�Ƶ��̳��� Address �� Mode ����
*/
void MediafarmLib::setArduinoConfig() {
	byte aPin,bPin,cPin,dPin= 0;

	aPin = digitalRead(4);//8
	bPin = digitalRead(5);//4
	cPin = digitalRead(6);//2
	dPin = digitalRead(7);//1

	arduino_id =(aPin*8)+(bPin*4)+(cPin*2)+dPin;

	arduino_mode = digitalRead(3);//mode
}




/*
�ø��� ����� ��ȿ�� üũ
*/
byte MediafarmLib::getCrcValue(byte checkArray[], int lastlength){
	byte crcValue = 0;
	for(int i = 0;i<lastlength;i++)
		crcValue += checkArray[i];  

	return crcValue;
}
