#ifndef IR_H
#define IR_H

#define IR1_TX_PORT      PORTB
#define IR1_TX_BIT       b0
#define IR2_TX_PORT      PORTA
#define IR2_TX_BIT       b12

#define IR1_TX_CONFIG     port_mode_out(IR1_TX_PORT,IR1_TX_BIT)
#define IR1_TX_ON         port_set     (IR1_TX_PORT,IR1_TX_BIT);
#define IR1_TX_OFF        port_reset   (IR1_TX_PORT,IR1_TX_BIT);

#define IR2_TX_CONFIG     port_mode_out(IR2_TX_PORT,IR2_TX_BIT)
#define IR2_TX_ON         port_set     (IR2_TX_PORT,IR2_TX_BIT);
#define IR2_TX_OFF        port_reset   (IR2_TX_PORT,IR2_TX_BIT);



void ir_init(void);




#endif
