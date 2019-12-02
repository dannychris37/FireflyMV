#include "fireflymv.h"

/** UDP SEND **/

int port = 3000;
const char *ip_send = "192.168.0.3"; //"192.168.7.50";
int socket_status, send_status;
struct sockaddr_in addr;

void UDPset(){

	// address family set to IPv4
    addr.sin_family = AF_INET;

    // convert IP addr from numbers-and-dots notation into binary
    // return 1 if succesful, 0 otherwise
    socket_status = inet_aton(ip_send, &addr.sin_addr);

    if (socket_status == 0){

        cout << "Error in coverting string address to binary number" << endl;
        exit(1);

    }

    // set port 
    addr.sin_port = htons(port);

    // create socket
    // PF_INET 		-> IP layer 3 protocol
    // SOCK_DGRAM	-> support datagrams
    // IPPROTO_UDP	-> UDP
    socket_status = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    if (socket_status == -1){

        cout<<"Error in socket creation "<< endl;

    }
}

void UDPfarewell(int markerID, Vec3d data, Vec3d rot){

    time_t t = time(0);
    double now = static_cast<double> (t);
    //cout<<"stamps:"<<now<< endl;
    
    //OpenCV vector of 8 doubles containing ArUco data
    Vec<double, 8> send_data{
    	now, 
    	double(markerID), 
    	data[0], 
    	data[1],
    	data[2], 
    	rot[0], 
    	rot[1], 
    	rot[2]
    }; 
    
    // Transmit message
    send_status = sendto(
    	socket_status,
    	&send_data,
    	sizeof(send_data),
    	0,
    	(struct sockaddr *)&addr, 
    	sizeof(addr)
    );

    if (send_status == -1){

        cout<< "error sending data"<<"\n error code: "<< send_status << endl;

    }
}