/***************************************************************************************************
A client that connects on the server and sends the XYZ command in order to control the motors thrust
***************************************************************************************************/
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayLayout.h>
#include "rossurface/ADCmcp3008Spi.h"
#include <wiringPi.h>
#define MOTEURSTART 300
#define MOTEURMAX   350
#define MOTEURMIN   250
#define MOTEURSTOP  300
#define CHANNEL0    0
using namespace std;



float rescale(float valeurAConvertir, float in_min, float in_max, float out_min, float out_max)
{
    if( valeurAConvertir < in_min) return out_min;
    if( valeurAConvertir > in_max) return out_max;
    return (valeurAConvertir - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void sleepMs(int ms)
{
    usleep(ms*1000); //convert to microseconds
    return;
}

int absoluteValue(int a )
{
    if(a<0 ) return -a ;
    return a ;
}

int isThereABigDifference(int a ,int b ,int thatIsABigDifference)
{
    if (absoluteValue(a-b)>thatIsABigDifference) return 1 ;
    return 0;
}

int main(int argc, char *argv[])
{

    char buffer[256];
    int valeurFin=205;
    int a2dVal[4] = {510,5110,510,0}; // destine a contenir les valeurs des axes X Y Z
    int a2dBuff[4] = {510,5110,510,0}; // destine a contenir les valeurs des axes X Y Z
    int a2dChannel = 0;
    unsigned char data[3];
    int channelActif=0;
    string coutRes[4]= {"x="," y="," Omega="," h= "};
    ADCmcp3008Spi joystickSpi("/dev/spidev0.0", SPI_MODE_0, 1000000, 8);
    std_msgs::Int32MultiArray tableauAEnvoyer ;


    ros::init(argc,argv,"commandeUtilisateur");

    ros::NodeHandle noeud;

    ros::Publisher publisherCommande = noeud.advertise<std_msgs::Int32MultiArray>("commandeUtilisateur",1000);

    ros::Rate loop_rate(100);

wiringPiSetupSys(); // Initialise les gpio en utilisant les nomnbres gpio
pinMode(17,INPUT);

pinMode(27,INPUT);

    /// Command loop

    while(ros::ok() )        ///Lorsque l'on tourne le 3eme axe , on "eteint le moteur"
    {
        tableauAEnvoyer.data.clear();
	system("clear");
for(a2dChannel=0; a2dChannel<4; a2dChannel++)
        {
            data[0] = 1;  //  first byte transmitted -> start bit
            data[1] = 0b10000000 |( ((a2dChannel & 7) << 4)); // second byte transmitted -> (SGL/DIF = 1, D2=D1=D0=0)
            data[2] = 0; // third byte transmitted....don't care
            joystickSpi.spiWriteRead(data, sizeof(data) );
            a2dBuff[a2dChannel] = 0;
            a2dBuff[a2dChannel] = (data[1]<< 8) & 0b1100000000; //merge data[1] & data[2] to get result
            a2dBuff[a2dChannel] |=  (data[2] & 0xff);
        
		if (a2dChannel == 3 ) 
		{
		
			switch ( digitalRead(17) + 2*digitalRead(27))
			{
			case 1 :
			        a2dBuff[3] = a2dBuff[3] ; 
			break ;
			case 2 :
			        a2dBuff[3] = - a2dBuff[3] ; 
			break ;
			default : 
			        a2dBuff[3] = 0 ;
			break;
			}

            a2dVal[a2dChannel] =  rescale( a2dBuff[a2dChannel] , -1023 , 1023 , -100 , 100 ) ;
		}
else 	 a2dVal[a2dChannel] =  rescale( a2dBuff[a2dChannel] , 100 , 900 , -100 , 100 ) ;

            cout <<coutRes[a2dChannel]<< a2dVal[a2dChannel]<<endl ;
	}
cout<<endl;

for(a2dChannel=0; a2dChannel<4; a2dChannel++)
{
   	 tableauAEnvoyer.data.push_back( a2dVal[a2dChannel] );
}

publisherCommande.publish(tableauAEnvoyer);
ros::spinOnce();

    }


    cout << "Client : adieu ros ! " << endl ;

    return 0;
}
