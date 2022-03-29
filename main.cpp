/*****************************************************************************************************\
 CUER BMU v3.0 Code by Rahul Swaminathan (July 2019)

 Functionality:

    Precharge: When main contactors are closed, close precharge relay and wait until DC bus is up to
    voltage. Once this is true, Engage HV Box contactor to provide current path for power electronics

    Discharge: When main contactors are opened, Open HV box contactor to isolate HV Box. Then, engage 
    discharge relay to discharge HV box capacitors to a safe voltage.

    Solar Relay Control: to be added in

    HV Box Fan Control: to be added in once we have a way of measuring HV box temperature

    Pack Fan Control: to be added in

    Cell temperature/voltage monitoring: Make decisions based off cell temperature and voltage
    readings (i.e. shut everything off if there a cell is over/under voltage/temperature) 

    IVT monitoring: Configures the IVT and monitors current; if max charging or discharging current
    is exceeded then shut everything off. 
\*****************************************************************************************************/

//TO DO: ADD TIMEOUT FOR CELL TEMPERATURE, IVT AND CELL VOLTAGE READINGS

#include <chrono>
#include <cstdio>
#include <mbed.h>

// DEBUG flag
#define BMU_DEBUG 1 

//definitions and i/o assignment
#define PRECHG_ENABLE p7
#define DISCHG_DISABLE p8
#define PRECHG_DETECT p15
#define HVDC_ENABLE p5

#define MAX_DISCHARGE_MAH 100000
#define MAX_CHARGE_MAH -100000

#define MAX_CELL_VOLTAGE 42000
#define MIN_CELL_VOLTAGE 30000
#define VOLTAGE_HYSTERESIS 100

#define MAX_CELL_TEMPERATURE 60
#define MIN_CELL_TEMPERATURE 1
#define TEMPERATURE_HYSTERESIS 2

#define CAN_TIMEOUT_MS 100
#define IVT_TIMEOUT_MS 1000

// Chrono-based elapsed_time for timer class
using namespace std::chrono;

/*****************************************************************************************************\
 CAN IDs
\*****************************************************************************************************/

//BMU heartbeat CAN ID
const int BMU_CAN_ID = 0x400;

//Driver Controls CAN ID
const int DRIVER_CONTROLS_ID = 0x500;

//PCU CAN IDs
const int CELL_VOLTAGES_01_04 = 0x360;
const int CELL_VOLTAGES_05_08 = 0x361;
const int CELL_VOLTAGES_09_12 = 0x362;
const int CELL_VOLTAGES_13_16 = 0x363;
const int CELL_VOLTAGES_17    = 0x364;
const int CELL_VOLTAGES_18_21 = 0x365;
const int CELL_VOLTAGES_22_25 = 0x366;
const int CELL_VOLTAGES_26_29 = 0x367;
const int CELL_VOLTAGES_30_33 = 0x368;
const int CELL_VOLTAGES_34    = 0x369;
const int PCU_STATUS_FRONT    = 0x340;
const int PCU_STATUS_REAR     = 0x341;

DigitalOut prechg_enable(PRECHG_ENABLE);
DigitalOut dischg_disable(DISCHG_DISABLE);
DigitalOut hvdc_enable(HVDC_ENABLE);
DigitalIn prechg_detect(PRECHG_DETECT);

DigitalOut voltage_led(LED2);
DigitalOut current_led(LED1);


//Serial port for debugging. Note that overuse of pc.printf() can mess up the CAN routines.
// Serial is deprecated in mbedos 6, use printf instead
// Serial pc(USBTX, USBRX);

//CAN setup
CAN can(p30, p29);
CANMessage received_msg;
int result;

//IVT variables stored in these 6 variables
int IVT_voltage;
int IVT_current;
int IVT_charge;
int IVT_temperature;
int IVT_power;
int IVT_energy;

//Cell voltages and temperatures stored in these arrays. We have more than enough RAM to do this.
uint16_t cell_voltages[34];
uint8_t cell_temperatures[34][6];

//Min and Max cell voltages, as well as voltage hysteresis, in 100μV. These don't change.
const int max_cell_voltage = MAX_CELL_VOLTAGE; 
const int min_cell_voltage = MIN_CELL_VOLTAGE;
const int voltage_hysteresis = VOLTAGE_HYSTERESIS;
// These are the values that do change due to hysteresis.
int temp_max_cell_voltage;
int temp_min_cell_voltage;

//Max allowable current, in mA. This doesn't change.
const int max_current = MAX_DISCHARGE_MAH;
const int max_charging_current = MAX_CHARGE_MAH; //this needs to be negative

//Max allowable cell (not IVT) temperature, in ˚C. These don't change.
const uint8_t max_cell_temperature = MAX_CELL_TEMPERATURE;
const uint8_t min_cell_temperature = MIN_CELL_TEMPERATURE;
const uint8_t temperature_hysteresis = TEMPERATURE_HYSTERESIS;
//These are the values that do change due to hysteresis.
uint8_t temp_min_cell_temperature;
uint8_t temp_max_cell_temperature;

//Function prototypes
void CANRecieveRoutine(void);
bool can_send(CANMessage msg);
void CANDataSentCallback(void);
void precharge(void);
void discharge(void);
void update_contactors(void);
void check_cells(void);
void update_BMU_status_array(void);
void config_IVT(void);
void set_heartbeat_flag(void);
void beat(void);
void print_bmu_status(void);

//Heartbeat ticker and various flags
Ticker heartbeat;
Timer IVT_timer;
unsigned long IVT_time;

bool CAN_data_sent;
bool heartbeat_flag;
bool error_flag;
bool over_voltage_flag;
bool under_voltage_flag;
bool over_temperature_flag;
bool under_temperature_flag;
bool ignition;
bool currently_precharging;
bool currently_discharging;

//various CAN messages set up as char arrays.
char contactor_array[1];
char BMU_status_array[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//IVT config messages
char stop_mode[5] = {0x34, 0x00, 0x00, 0x00, 0x00};
char start_mode[5] = {0x34, 0x01, 0x01, 0x00, 0x00};
char IVT_current_setup[4] = {0x20, 0x02, 0x00, 0x19};
char IVT_voltage1_setup[4] = {0x21, 0x02, 0x03, 0xE8};
char IVT_voltage2_setup[4] = {0x22, 0x00, 0x03, 0xE8};
char IVT_voltage3_setup[4] = {0x23, 0x00, 0x03, 0xE8};
char IVT_temperature_setup[4] = {0x24, 0x02, 0x03, 0xE8};
char IVT_charge_setup[4] = {0x25, 0x02, 0x03, 0xE8};
char IVT_power_setup[4] = {0x26, 0x02, 0x03, 0xE8};
char IVT_energy_setup[4] = {0x27, 0x02, 0x03, 0xE8};

CANMessage stop_msg(0x411, stop_mode, 5);
CANMessage start_msg(0x411, start_mode, 5);
CANMessage current_setup_msg(0x411, IVT_current_setup, 4);
CANMessage voltage1_setup_msg(0x411, IVT_voltage1_setup, 4);
CANMessage voltage2_setup_msg(0x411, IVT_voltage2_setup, 4);
CANMessage voltage3_setup_msg(0x411, IVT_voltage3_setup, 4);
CANMessage temperature_setup_msg(0x411, IVT_temperature_setup, 4);
CANMessage charge_setup_msg(0x411, IVT_charge_setup, 4);
CANMessage power_setup_msg(0x411, IVT_power_setup, 4);
CANMessage energy_setup_msg(0x411, IVT_energy_setup, 4);

//A struct to contain all the stuff the BMU puts in its heartbeat
struct BMU_status {
    bool over_current;
    bool under_voltage;
    bool over_voltage;
    bool under_temperature;
    bool over_temperature;
    bool safe_to_drive;
    bool charging_state;
    bool precharge_state;
    bool discharge_state;
    bool contactor_state;
    uint8_t fan1_state;
    uint8_t fan2_state;
    uint8_t fan3_state;
    uint8_t fan4_state;
} BMU;

//This is used to store the error flags; you'll see its use later in the main loop.
char previous_status = 0x00;

int main(void) {
    //Initialise the BMU with all the error flags set for safety, and the safe to drive flag cleared
    BMU.over_voltage = 1;
    BMU.under_voltage = 1;
    BMU.over_current = 1;
    //BMU.under_temperature = 0;
    //BMU.over_temperature = 1;
    BMU.safe_to_drive = 0;

    //Attach the ticker to set_heartbeat_flag() at a rate of 1Hz.
    heartbeat.attach(&set_heartbeat_flag, 1000ms);

    //Setup the CAN and attach it to the receive routine.
    can.frequency(500000);
    can.attach(&CANRecieveRoutine, CAN::RxIrq);
    can.attach(&CANDataSentCallback, CAN::TxIrq);

    IVT_timer.start();

    while(1) {
        //Check the cell voltages, temperatures, and current and update the BMU status array to be sent over CAN
        check_cells();
        update_BMU_status_array();

        //We want to send the BMU status every second when there are no errors
        //When there is a new error, immediately send the BMU status, then keep sending it every second
        if(heartbeat_flag) {
            heartbeat_flag = false;
            beat();
        }
        if(error_flag) {
            if(previous_status != BMU_status_array[0])
                beat();
        }
        
        //Store the previous BMU status to prevent the same error rapidly triggering CAN messages to be sent
        previous_status = BMU_status_array[0]; 
    }
}

/*****************************************************************************************************\
 The most important function: the CAN message received interrupt callback.
 Tells the BMU what to do with each different message ID in a big switch statement 
\*****************************************************************************************************/
void CANRecieveRoutine (void) {
    result = can.read(received_msg);
    switch(received_msg.id) {
    //cases 0x320 - 0x329 are cell voltage readings from the PCU.
    //If time permits can shorten this to a "case 0x320 ... 0x323, case 0x324, case 0x325 ... 0x328, case 0x329"
    case CELL_VOLTAGES_01_04:
        cell_voltages[0] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        cell_voltages[1] = (received_msg.data[2]) | (received_msg.data[3] << 8);
        cell_voltages[2] = (received_msg.data[4]) | (received_msg.data[5] << 8);
        cell_voltages[3] = (received_msg.data[6]) | (received_msg.data[7] << 8);
        break;
    case CELL_VOLTAGES_05_08:
        cell_voltages[4] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        cell_voltages[5] = (received_msg.data[2]) | (received_msg.data[3] << 8);
        cell_voltages[6] = (received_msg.data[4]) | (received_msg.data[5] << 8);
        cell_voltages[7] = (received_msg.data[6]) | (received_msg.data[7] << 8);
        break;
    case CELL_VOLTAGES_09_12:
        cell_voltages[8] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        cell_voltages[9] = (received_msg.data[2]) | (received_msg.data[3] << 8);
        cell_voltages[10] = (received_msg.data[4]) | (received_msg.data[5] << 8);
        cell_voltages[11] = (received_msg.data[6]) | (received_msg.data[7] << 8);
        break;
    case CELL_VOLTAGES_13_16:
        cell_voltages[12] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        cell_voltages[13] = (received_msg.data[2]) | (received_msg.data[3] << 8);
        cell_voltages[14] = (received_msg.data[4]) | (received_msg.data[5] << 8);
        cell_voltages[15] = (received_msg.data[6]) | (received_msg.data[7] << 8);
        break;
    case CELL_VOLTAGES_17:
        cell_voltages[16] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        break;
    case CELL_VOLTAGES_18_21:
        cell_voltages[17] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        cell_voltages[18] = (received_msg.data[2]) | (received_msg.data[3] << 8);
        cell_voltages[19] = (received_msg.data[4]) | (received_msg.data[5] << 8);
        cell_voltages[20] = (received_msg.data[6]) | (received_msg.data[7] << 8);
        break;
    case CELL_VOLTAGES_22_25:
        cell_voltages[21] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        cell_voltages[22] = (received_msg.data[2]) | (received_msg.data[3] << 8);
        cell_voltages[23] = (received_msg.data[4]) | (received_msg.data[5] << 8);
        cell_voltages[24] = (received_msg.data[6]) | (received_msg.data[7] << 8);
        break;
    case CELL_VOLTAGES_26_29:
        cell_voltages[25] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        cell_voltages[26] = (received_msg.data[2]) | (received_msg.data[3] << 8);
        cell_voltages[27] = (received_msg.data[4]) | (received_msg.data[5] << 8);
        cell_voltages[28] = (received_msg.data[6]) | (received_msg.data[7] << 8);
        break;
    case CELL_VOLTAGES_30_33:
        cell_voltages[29] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        cell_voltages[30] = (received_msg.data[2]) | (received_msg.data[3] << 8);
        cell_voltages[31] = (received_msg.data[4]) | (received_msg.data[5] << 8);
        cell_voltages[32] = (received_msg.data[6]) | (received_msg.data[7] << 8);
        break;
    case CELL_VOLTAGES_34:
        cell_voltages[33] = (received_msg.data[0]) | (received_msg.data[1] << 8);
        break;  
    //Ignition message received by the Driver Controls board 
    case DRIVER_CONTROLS_ID:
        ignition = (received_msg.data[0] & 0x01);
        break;
    //Messages 0x521 - 0x528 are IVT messages.
    case 0x521:
        IVT_current = (received_msg.data[5]) | (received_msg.data[4] << 8) | (received_msg.data[3] << 16) | (received_msg.data[2] << 24);
        IVT_timer.stop();
        IVT_time = duration_cast<milliseconds>(IVT_timer.elapsed_time()).count();
        IVT_timer.reset();
        IVT_timer.start();
        break;
    case 0x522:
        IVT_voltage = (received_msg.data[5]) | (received_msg.data[4] << 8) | (received_msg.data[3] << 16) | (received_msg.data[2] << 24);
        break;
    //We don't want U2 and U3 voltage readings. If the IVT is sending these (it always will when restarted) we need to configure it
    case 0x523 ... 0x524:
        config_IVT();
        break;
    case 0x525:
        IVT_temperature = (received_msg.data[5]) | (received_msg.data[4] << 8) | (received_msg.data[3] << 16) | (received_msg.data[2] << 24);
        break;
    case 0x526:
        IVT_power = (received_msg.data[5]) | (received_msg.data[4] << 8) | (received_msg.data[3] << 16) | (received_msg.data[2] << 24);
        break;
    case 0x527:
        IVT_charge = (received_msg.data[5]) | (received_msg.data[4] << 8) | (received_msg.data[3] << 16) | (received_msg.data[2] << 24);
        break;
    case 0x528:
        IVT_energy = (received_msg.data[5]) | (received_msg.data[4] << 8) | (received_msg.data[3] << 16) | (received_msg.data[2] << 24);
        break;  
    //Cell temperature messages
    case 0x550 ... 0x572:
        for(int i = 0; i < 6; i++) {
            cell_temperatures[received_msg.id - 0x550][i] = received_msg.data[i];
        }
    }
}

bool can_send(CANMessage msg){
    Timer t;
    CAN_data_sent = false;
    t.start();
    can.write(msg);
    // Check whether elapsed time has exceeded CAN_TIMEOUT_MS. If it has, return false
    while (!CAN_data_sent && (duration_cast<milliseconds>(t.elapsed_time()).count()) <  milliseconds(CAN_TIMEOUT_MS).count());
    return duration_cast<milliseconds>(t.elapsed_time()).count() <= milliseconds(CAN_TIMEOUT_MS).count();
}

void CANDataSentCallback(void){
    CAN_data_sent = true;
}

/*****************************************************************************************************\
 A function to configure the IVT. We put it in stop mode, write the set commands, and put it back
 into start mode.
\*****************************************************************************************************/
void config_IVT(void) {
    can_send(stop_msg);
    // Wait 50 microseconds
    wait_us(50);
    can_send(current_setup_msg);
    wait_us(50);
    can_send(voltage1_setup_msg);
    wait_us(50);
    can_send(voltage2_setup_msg);
    wait_us(50);
    can_send(voltage3_setup_msg);
    wait_us(50);
    can_send(temperature_setup_msg);
    wait_us(50);
    can_send(charge_setup_msg);
    wait_us(50);
    can_send(power_setup_msg);
    wait_us(50);
    can_send(energy_setup_msg);
    wait_us(50);
    can_send(start_msg);
    wait_us(50);
}

/*****************************************************************************************************\
 This is attached to the ticker; we can't write/read CAN inside an interrupt so we just set a flag
 and call a function whenever the flag is set.
\*****************************************************************************************************/
void set_heartbeat_flag(void) {
    heartbeat_flag = true;
}

/*****************************************************************************************************\
 The actual function we want to call whenever the ticker is triggered. This sends a BMU status message
 as well as updates contactor states.
\*****************************************************************************************************/
void beat(void) {
    // If debug mode is on, print BMU status over serial
    if (BMU_DEBUG)
    {
        print_bmu_status();
    }
    //BMU will send a CAN message containing status messages
    CANMessage BMU_status_msg(BMU_CAN_ID, BMU_status_array, 6);
    can_send(BMU_status_msg);

    update_contactors();
}

/*****************************************************************************************************\
 Precharge routine whenever the car is turned on: connect the motor controller across the precharge
 resistor, and once up to voltage close the main contactor and disconnect the precharge resistor.
 This requires the PCU contactor to be on. 
\*****************************************************************************************************/
void precharge(void) {
    //If we're precharging, then we're no longer discharged.
    BMU.discharge_state = false;
    //A flag to say we're currently precharging
    currently_precharging = true;
    //The discharge relay should already be open, but just to make sure we open it again
    dischg_disable = 1;
    //close precharge relay
    prechg_enable = 1;
    //Small 0.5s for safety, then wait until there's no more current flowing through the precharge resistor
    // However, in general it is not a good practice to wait for that long in ISR
    wait_us(500000);
    //This is a place where it's possible for the BMU to freeze up
    while(!prechg_detect);
    //close the HV box contactor and open the precharge relay
    hvdc_enable = 1;
    // Wait for 0.1 seconds
    wait_us(100000);
    prechg_enable = 0;
    //We are now no longer precharging
    currently_precharging = false;
    //a flag to make sure we don't precharge again if already precharged
    //this flag will only be cleared upon discharging
    BMU.precharge_state = true;
}

/*****************************************************************************************************\
 Discharge routine whenever the car is turned off (either manually or due to an error):
 Open the HV box contactor and close the discharge relay
\*****************************************************************************************************/
void discharge(void) {
    //If we're discharging we're no longer precharged
    BMU.precharge_state = false;
    //A flag to say we're currently discharging
    currently_discharging = true;
    //The precharge relay should already be open, but just to make sure we open it again
    prechg_enable = 0;
    //Open the HV box contactor and close the discharge relay
    hvdc_enable = 0;
    // Wait for 0.1s, not a good practice to wait for that long in ISR
    wait_us(100000);
    dischg_disable = 0;
    //Might want to add a delay here as we don't have a discharge detect; can measure the time it takes to discharge the HV caps and use that value
    //Without that delay it will look like the discharge process is instantaneous
    currently_discharging = false;
    BMU.discharge_state = true;
}

/*****************************************************************************************************\
 A function that turns on the contactors if the ignition is on and safe, and turns them off otherwise.
 This is only called in the beat() function but I've kept it separate so we can call it on its own
 if desired.
\*****************************************************************************************************/
void update_contactors(void) {
    //Self explanatory, if the car is on and it's safe then turn on contactors & precharge if needed
    if(ignition == 1 && BMU.safe_to_drive) {
        if (BMU_DEBUG)
        {
            printf("Contactors engaged. \n");
        }
        contactor_array[0] = 0x01;
        CANMessage contactor_msg(0x34F, contactor_array, 1);
        can_send(contactor_msg);
        if(!BMU.precharge_state)
        {
            if (BMU_DEBUG)
            {
                printf("Start precharge. \n");
            }
            precharge();
        }
    }

    //In any other scenario, turn off the contactors and discharge
    else {
        if (BMU_DEBUG)
        {
            printf("Contactors disengaged. \n");
        }
        contactor_array[0] = 0x00;
        CANMessage contactor_msg(0x34F, contactor_array, 1);
        can_send(contactor_msg);
        if(!BMU.discharge_state)
        {
            if (BMU_DEBUG)
            {
                printf("Start discharge. \n");
            }
            discharge();
        }
    }
}

/*****************************************************************************************************\
 A function that checks cell voltages, temperatures, and IVT current to make sure they are within the
 operating limits. This updates the BMU struct but NOT the BMU status array to be sent over CAN.
\*****************************************************************************************************/
void check_cells(void) {
    //Check whether we are charging
    if(IVT_current < 0)
        BMU.charging_state = true;
    else
        BMU.charging_state = false;
    //Check the max current isn't exceeded in both charging and discharging directions
    if(IVT_current >= max_current || IVT_current < max_charging_current) {
        BMU.over_current = true;
        current_led = 1;
    }
    else {
        BMU.over_current = false;
        current_led = 0;
    }
    //This bit is for the subsequent iterations of the code, to add some hysteresis into the under/over-voltage triggers
    if(over_voltage_flag)
        temp_max_cell_voltage = max_cell_voltage - voltage_hysteresis;
    else
        temp_max_cell_voltage = max_cell_voltage;

    if(under_voltage_flag)
        temp_min_cell_voltage = min_cell_voltage + voltage_hysteresis;
    else
        temp_min_cell_voltage = min_cell_voltage;
    //Clear these flags before checking the cells, set them if we detect an over/under-voltage cell
    over_voltage_flag = false;
    under_voltage_flag = false;

    for(int i = 0; i < 17; i++) {
        if(cell_voltages[i] < temp_min_cell_voltage) {
            BMU.under_voltage = true;
            under_voltage_flag = true;
        }
        if(cell_voltages[i] > temp_max_cell_voltage) {
            BMU.over_voltage = true;
            over_voltage_flag = true;
        }
    }

    //If none of the cells are over/under-voltage, clear the BMU struct under/over-voltage flags
    if(over_voltage_flag == false && under_voltage_flag == false) {
        BMU.under_voltage = false;
        BMU.over_voltage = false;
    }
    
    //Same thing as the voltage, except now for temperature.
    if(over_temperature_flag)
        temp_max_cell_temperature = max_cell_temperature - temperature_hysteresis;
    else
        temp_max_cell_temperature = max_cell_temperature;

    if(under_temperature_flag)
        temp_min_cell_temperature = min_cell_temperature + temperature_hysteresis;
    else
        temp_min_cell_temperature = min_cell_temperature;

    over_temperature_flag = false;
    under_temperature_flag = false;

    for(int i = 0; i < 1; i++) { //i should go up to 34 in the final, just checking the first one for the test rig 
        for(int j = 0; j < 1; j++) { //j should go up to 6 in the final, just checking the first one for the test rig
            if(cell_temperatures[i][j] < temp_min_cell_temperature) {
                BMU.under_temperature = true;
                under_temperature_flag = true;
            }
            if(cell_temperatures[i][j] > temp_max_cell_temperature) {
                BMU.over_temperature = true;
                over_temperature_flag = true;
            }
        }
    }

    if(over_temperature_flag == false && under_temperature_flag == false) {
        BMU.under_temperature = false;
        BMU.over_temperature = false;
    }
}

/*****************************************************************************************************\
 This function checks the BMU struct for various flags and updates the BMU status array (the bytes
 sent over CAN) accordingly.
\*****************************************************************************************************/
void update_BMU_status_array(void) {
    //This flag is there to trigger beat() outside the ticker as soon as an error is detected
    error_flag = false;
    if(IVT_time > milliseconds(IVT_TIMEOUT_MS).count())
        error_flag = true;
    //Check current
    if(BMU.over_current) {
        BMU_status_array[0] |= 1<<0;
        error_flag = true;
    }
    else
        BMU_status_array[0] &= ~(1<<0);
    //Check under-voltage
    if(BMU.under_voltage) {
        BMU_status_array[0] |= 1<<1;
        error_flag = true;
    }
    else
        BMU_status_array[0] &= ~(1<<1);
    //Check over-voltage
    if(BMU.over_voltage) {
        BMU_status_array[0] |= 1<<2;
        error_flag = true;        
    }
    else
        BMU_status_array[0] &= ~(1<<2);
    //Check under-temperature
    if(BMU.under_temperature) {
        BMU_status_array[0] |= 1<<3;
        error_flag = true;
    }
    else
        BMU_status_array[0] &= ~(1<<3);
    //Check over-temperature
    if(BMU.over_temperature) {
        BMU_status_array[0] |= 1<<4;
        error_flag = true;
    }
    else
        BMU_status_array[0] &= ~(1<<4);
    //If there was an error, turn off the ignition and clear the safe to drive flag
    if(error_flag) {
        BMU.safe_to_drive = false;
        ignition = false;
        BMU_status_array[0] &= ~(1<<5);
    }
    //If no errors, tell us it's safe to drive
    else {
        BMU.safe_to_drive = true;
        BMU_status_array[0] |= 1<<5;
    }
    //Charging flag
    if(BMU.charging_state)
        BMU_status_array[1] |= 1<<0;
    else
        BMU_status_array[1] &= ~(1<<0);
    //Precharge and Discharge flags
    if(BMU.precharge_state)
        BMU_status_array[1] |= 1<<1;
    else
        BMU_status_array[1] &= ~(1<<1);
    if(BMU.discharge_state)
        BMU_status_array[1] |= 1<<2;
    else
        BMU_status_array[1] &= ~(1<<2);

    //Placeholder for fan states, code setting BMU.fanx_state needs to be updated
    BMU_status_array[2] = BMU.fan1_state;
    BMU_status_array[3] = BMU.fan2_state;
    BMU_status_array[4] = BMU.fan3_state;
    BMU_status_array[5] = BMU.fan4_state;
}

/*****************************************************************************************************\
 This function prints the content of BMU struct over serial to show the status of the BMU. It is only
 used for debugging purposes.
\*****************************************************************************************************/
void print_bmu_status(void)
{
    printf("BMU status \n");
    printf("==================================== \n");
    printf("over_current: %d \n", BMU.over_current);
    printf("under_voltage: %d \n", BMU.under_voltage);
    printf("under_temperature: %d \n", BMU.under_temperature);
    printf("over_temperature: %d \n", BMU.over_temperature);
    printf("safe_to_drive: %d \n", BMU.safe_to_drive);
    printf("charging_state: %d \n", BMU.charging_state);
    printf("precharge_state: %d \n", BMU.precharge_state);
    printf("discharge_state: %d \n", BMU.discharge_state);
    printf("contactor_state: %d \n", BMU.contactor_state);
    printf("\n");
}