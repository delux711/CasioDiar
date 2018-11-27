#ifndef __casioDiar_H
#define __casioDiar_H

#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>

typedef enum _cd_state_e {
    CD_STATE_NOT_INIT,
    CD_STATE_SLEEP,
    CD_STATE_SENDING,
    CD_STATE_ERROR,
    CD_STATE_RECEIVING,
    CD_STATE_SENDED_CALENDAR,
    CD_STATE_SENDED_TELEPHONE,
    CD_STATE_SENDED_NOTE,
    CD_STATE_SENDED_SCHEDULE,
    CD_STATE_SENDED_REMINDER,
    CD_STATE_SENDED_REMINDER2,
    CD_STATE_SENDED_FREE_FILE,
    CD_STATE_SENDED_DATA
} cd_state_e;


extern cd_state_e CD_task(void);
extern uint8_t CD_receive(void);
extern void CD_sendToDiarConst(uint8_t *buff);
extern void CD_senToDiarEndCommunication(void);
extern uint8_t *CD_getBuffer(void);

/*
The general field format for all fields in a record is:
    :llffoottdd...ddcc

where:
:  - Field always begins with leading colon

ll  - 2 digit (1-byte) field data length, number of bytes
      of data (dd) in field, not counting llffoott and cc

      Maximum value is 80 hex (128)

      Zero length always indicates a record terminating field
---------------------------------------------------------------------------------------------------------------
ff  - Field data type:                                   |                                                    |
                                                         |                                                    |
        0x - type identification field or                |  9x - free file name (SF-R/M series only)          |
             record terminating field                    |  7x - other data (SF-R/M series and NX/CSF series) |
        8x - text data field or MARK terminating field   |       70 = free file new/repeat flag               |
        Cx - schedule alarm field                        |       71 = color data (R/G/B)                      |
        Dx - calendar field                              |       72 = ToDo priority A/B/C                     |
        Ex - time field                                  |       78 = calendar colours                        |
        Fx - date field                                  |  2x - other data (NX series, CSF series)           |
                                                         |       21 = icon number                             |
                                                         |       22 = sketch notebook number (NX series only) |
                                                         |  4x - sketch data (NX series only)                 |
                                                         |----------------------------------------------------|
      The low digit is used as a bit flag on later series, for various sub-codes as noted.                    |
                                                                                                              |
    - Low digit of flag byte should be masked to 0 when read to mask out offset bit                           |
---------------------------------------------------------------------------------------------------------------
oo  - Low byte of data offset (bits 0-7)

      High bit of data offset (bit 8) is in low bit of flag byte
---------------------------------------------------------------------------------------------------
tt  - Field type,                                       |   80  - calendar                        |
            01 for record terminator field,  ___________|   90  - telephone                       |
            02 for type identification field,___________    A0  - memo                            |
            FF for transmission terminator field,       |   B0  - schedule                        |
            00 for all data fields                      |   C0  - business card (some models)     |
--------------------------------------------------------|   C1  - todo list (later models)        |
dd  - One or more data bytes (counted in length field)  |   91  - reminder (later models)         |
                                                        |   A1  - reminder 2 (SF-R/M series only) |
cc  - Checksum                                          |   92  - expense (later models)          |
                                                        |   B1  - free file (SF-R/M series only)  |
:llffoottddddcc                                         |   A2  - sketch data (NX series          |
:02000002A0005C                                         -------------------------------------------
  ||   | |   |
  ||   | |   checksum 02+00+00+02+A0+00+5C = x00
  ||   | |
  ||   | type A0 is memo
  ||   |
  ||   field type 02 indicates type identification field
  ||
  |field data type always 00
  |
  data length always 02 (the 2 bytes A000 in this record)

Text data fields always have a field data type of 8xh.
Example of text data field "1234":
    :llffoottddddddddcc
  :0480000031323334B2
    || |   |       |
    || |   |       checksum 04+80+00+00+31+32+33+34+B2 = x00
    || |   |
    || |   data bytes for entry field "1234"
    || |
    || offset is 0 for first entry
    ||
    |field data type 80 indicates text data field
    |
    field length is 4 data bytes
  
***************************************************************************************************
Date field format:                            Time field format:
                                              
  :0AFx0000yyyyyyyy2Dmmmm2Dddddcc               :05Ex0000hhhh3Ammmmmcc
           |       | |   | |                             |   | |
           |       | |   | day (01-1F)                   |   | minute (00-3B)
           |       | |   |                               |   |
           |       | |   dash (-)                        |   colon :
           |       | |                                   |
           |       | month (01-0C)                       hour (01-18)
           |       |
           |       dash (-)
           |
           year (19xx or 20xx)
         
***************************************************************************************************
For a schedule appointment with time duration specified (start and endtimes):

  :0BEx0000hhhh3Ammmmm7Ehhhh3Ammmmcc
           |           ||
           |           |end time (hh:mm)
           |           |
           |           separator character ~
           |
           start time (hh:mm)

Term schedule items (SF-9000 series):
  :15Fx0000yyyyyyyy2Dmmmm2Ddddd7Eyyyyyyyy2Dmmmm2Dddddcc
           |       | |   | |   | |       | |   | |
           |       | |   | |   | |       | |   | end day (01-1F)
           |       | |   | |   | |       | |   |
           |       | |   | |   | |       | |   dash (-)
           |       | |   | |   | |       | |
           |       | |   | |   | |       | end month (01-0C)
           |       | |   | |   | |       |
           |       | |   | |   | |       dash (-)
           |       | |   | |   | |
           |       | |   | |   | end year (19xx or 20xx)
           |       | |   | |   |
           |       | |   | |   separator character ~
           |       | |   | |
           |       | |   | start day (01-1F)
           |       | |   |
           |       | |   dash (-)
           |       | |
           |       | start month (01-0C)
           |       |
           |       dash (-)
           |
           start year (19xx or 20xx)
*/
#endif
