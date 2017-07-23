/* mbed Microcontroller Library
 * Copyright (c) 2017 u-blox
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file gnss.cpp
 * This file defines a class that communicates with a u-blox GNSS chip.
 */

#include "mbed.h"
#include "ctype.h"
#include "gnss.h"

GnssParser::GnssParser(void)
{
    // Create the enable pin but set everything to disabled
    _gnssEnable = NULL;
    
#if defined GNSSEN && defined TARGET_UBLOX_C030 /* TODO  */
    _gnssEnable = new DigitalInOut(GNSSEN, PIN_OUTPUT, PushPullNoPull, 0);
#endif
}

GnssParser::~GnssParser(void)
{
    if (_gnssEnable != NULL) {
        *_gnssEnable = 0;
        delete _gnssEnable;
    }
}

void GnssParser::powerOff(void)
{
    // set the GNSS into backup mode using the command RMX-LPREQ
    struct { unsigned long dur; unsigned long flags; } msg = {0/*endless*/,0/*backup*/};
    sendUbx(0x02, 0x41, &msg, sizeof(msg));
}

void GnssParser::_powerOn(void)
{
    if (_gnssEnable != NULL) {
       *_gnssEnable = 1;
    }
    wait_ms (1);
}

int GnssParser::_getMessage(Pipe<char>* pipe, char* buf, int len)
{
    int unkn = 0;
    int sz = pipe->size();
    int fr = pipe->free();
    if (len > sz)
        len = sz;
    while (len > 0)
    {
        // NMEA protocol
        pipe->set(unkn);
        int nmea = _parseNmea(pipe,len);
        if ((nmea != NOT_FOUND) && (unkn > 0))  
            return UNKNOWN | pipe->get(buf,unkn);
        if (nmea == WAIT && fr)                       
            return WAIT;
        if (nmea > 0)                           
            return NMEA | pipe->get(buf,nmea);
        // UBX protocol
        
        pipe->set(unkn);
        int ubx = _parseUbx(pipe,len);
        if ((ubx != NOT_FOUND) && (unkn > 0))   
            return UNKNOWN | pipe->get(buf,unkn);
        if (ubx == WAIT && fr)                        
            return WAIT;
        if (ubx > 0)                            
            return UBX | pipe->get(buf,ubx);
        
        // UNKNOWN
        unkn ++;
        len--;
    }
    if (unkn > 0)                      
        return UNKNOWN | pipe->get(buf,unkn); 
    return WAIT;
}

int GnssParser::_parseNmea(Pipe<char>* pipe, int len)
{
    int o = 0;
    int c = 0;
    char ch;
    if (++o > len)                      return WAIT;
    if ('$' != pipe->next())            return NOT_FOUND;
    // this needs to be extended by crc checking 
    for (;;)
    {
        if (++o > len)                  return WAIT;
        ch = pipe->next();
        if ('*' == ch)                  break; // crc delimiter 
        if (!isprint(ch))               return NOT_FOUND; 
        c ^= ch;
    }
    if (++o > len)                      return WAIT;
    ch = _toHex[(c >> 4) & 0xF]; // high nibble
    if (ch != pipe->next())             return NOT_FOUND;
    if (++o > len)                      return WAIT;
    ch = _toHex[(c >> 0) & 0xF]; // low nibble
    if (ch != pipe->next())             return NOT_FOUND;
    if (++o > len)                      return WAIT;
    if ('\r' != pipe->next())           return NOT_FOUND;
    if (++o > len)                      return WAIT;
    if ('\n' != pipe->next())           return NOT_FOUND;
    return o;
}

int GnssParser::_parseUbx(Pipe<char>* pipe, int l)
{
    int o = 0;
    if (++o > l)                return WAIT;
    if ('\xB5' != pipe->next()) return NOT_FOUND;   
    if (++o > l)                return WAIT;
    if ('\x62' != pipe->next()) return NOT_FOUND;
    o += 4;
    if (o > l)                  return WAIT;
    int i,j,ca,cb;
    i = pipe->next(); ca  = i; cb  = ca; // cls
    i = pipe->next(); ca += i; cb += ca; // id
    i = pipe->next(); ca += i; cb += ca; // len_lsb
    j = pipe->next(); ca += j; cb += ca; // len_msb 
    j = i + (j << 8);
    while (j--)
    {
        if (++o > l)            return WAIT;
        i = pipe->next(); 
        ca += i; 
        cb += ca;
    }
    ca &= 0xFF; cb &= 0xFF;
    if (++o > l)                return WAIT;
    if (ca != pipe->next())     return NOT_FOUND;
    if (++o > l)                return WAIT;
    if (cb != pipe->next())     return NOT_FOUND;
    return o;
}

int GnssParser::send(const char* buf, int len)
{
    return _send(buf, len);
}

int GnssParser::sendNmea(const char* buf, int len)
{
    char head[1] = { '$' };
    char tail[5] = { '*', 0x00/*crc_high*/, 0x00/*crc_low*/, '\r', '\n' };
    int i;
    int crc = 0;
    for (i = 0; i < len; i ++)
        crc ^= *buf++;
    i  = _send(head, sizeof(head));
    i += _send(buf, len);
    tail[1] = _toHex[(crc > 4) & 0xF0];
    tail[2] = _toHex[(crc > 0) & 0x0F];
    i += _send(tail, sizeof(tail));
    return i;
}

int GnssParser::sendUbx(unsigned char cls, unsigned char id, const void* buf /*= NULL*/, int len /*= 0*/)
{
    char head[6] = { 0xB5, 0x62, cls, id, (char) len, (char) (len >> 8)};
    char crc[2];
    int i;
    int ca = 0;
    int cb = 0;
    for (i = 2; i < 6; i ++)
    {
        ca += head[i];
        cb += ca;
    }
    for (i = 0; i < len; i ++)
    {
        ca += ((char*)buf)[i];
        cb += ca; 
    }
    i  = _send(head, sizeof(head));
    i += _send(buf, len);
    crc[0] = ca & 0xFF;
    crc[1] = cb & 0xFF;
    i += _send(crc,  sizeof(crc));
    return i;
}

const char* GnssParser::findNmeaItemPos(int ix, const char* start, const char* end)
{
    // find the start
    for (; (start < end) && (ix > 0); start ++)
    {
        if (*start == ',')
            ix --;
    }
    // found and check bounds
    if ((ix == 0) && (start < end) && 
        (*start != ',') && (*start != '*') && (*start != '\r') && (*start != '\n'))
        return start;
    else 
        return NULL;
}

bool GnssParser::getNmeaItem(int ix, char* buf, int len, double& val)
{
    char* end = &buf[len];
    const char* pos = findNmeaItemPos(ix, buf, end);
    // find the start
    if (!pos)
        return false;
    val = strtod(pos, &end);
    // restore the last character
    return (end > pos);
}

bool GnssParser::getNmeaItem(int ix, char* buf, int len, int& val, int base /*=10*/)
{
    char* end = &buf[len];
    const char* pos = findNmeaItemPos(ix, buf, end);
    // find the start
    if (!pos)
        return false;
    val = (int)strtol(pos, &end, base);
    return (end > pos);
}

bool GnssParser::getNmeaItem(int ix, char* buf, int len, char& val)
{
    const char* end = &buf[len];
    const char* pos = findNmeaItemPos(ix, buf, end);
    // find the start
    if (!pos)
        return false;
    // skip leading spaces
    while ((pos < end) && isspace(*pos))
        pos++;
    // check bound
    if ((pos < end) && 
        (*pos != ',') && (*pos != '*') && (*pos != '\r') && (*pos != '\n'))
    {
        val = *pos;
        return true;
    }
    return false;
}

bool GnssParser::getNmeaAngle(int ix, char* buf, int len, double& val)
{
    char ch;
    if (getNmeaItem(ix,buf,len,val) && getNmeaItem(ix+1,buf,len,ch) && 
        ((ch == 'S') || (ch == 'N') || (ch == 'E') || (ch == 'W')))
    {
        val *= 0.01;
        int i = (int)val;
        val = (val - i) / 0.6 + i;
        if (ch == 'S' || ch == 'W')
            val = -val;
        return true;
    }
    return false;
}
                
const char GnssParser::_toHex[] = { '0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F' };

// ----------------------------------------------------------------
// Serial Implementation 
// ----------------------------------------------------------------

GnssSerial::GnssSerial(PinName tx /*= GNSSTXD  */, PinName rx /*= GNSSRXD */, int baudrate /*= GNSSBAUD */,
            int rxSize /*= 256 */, int txSize /*= 128 */) :
            SerialPipe(tx, rx, baudrate, rxSize, txSize)
{
    baud(baudrate);
}

GnssSerial::~GnssSerial(void)
{
    powerOff();
}

bool GnssSerial::init(PinName pn)
{
    // Power up and enable the module
    _powerOn();

    // send a byte to wakup the device again
    putc(0xFF);
    // wait until we get some bytes
    int size = _pipeRx.size();
    Timer timer;
    timer.start();
    while ((100 > timer.read_ms()) && (size == _pipeRx.size()))
        /* nothing / just wait */;
    return (size != _pipeRx.size());
}

int GnssSerial::getMessage(char* buf, int len)
{
    return _getMessage(&_pipeRx, buf, len);   
}

int GnssSerial::_send(const void* buf, int len)
{ 
    return put((const char*)buf, len, true/*=blocking*/); 
}

// ----------------------------------------------------------------
// I2C Implementation 
// ----------------------------------------------------------------

GnssI2C::GnssI2C(PinName sda /*= NC */, PinName scl /*= NC */,
               unsigned char i2cAdr /*= (66<<1) */, int rxSize /*= 256 */) :
               I2C(sda,scl),
               _pipe(rxSize),
               _i2cAdr(i2cAdr)
{
    frequency(100000);
}

GnssI2C::~GnssI2C(void)
{
    powerOff();
}

bool GnssI2C::init(PinName pn)
{
    // Power up and enable the module
    _powerOn();

    if (pn != NC) {
        DigitalOut pin(pn, 0);
        ::wait_us(1);
        pin = 1;
        ::wait_ms(100);
    }
    return !I2C::write(_i2cAdr,&REGSTREAM,sizeof(REGSTREAM));
}

int GnssI2C::getMessage(char* buf, int len)
{
    // fill the pipe
    int sz = _pipe.free();
    if (sz) 
        sz = _get(buf, sz);
    if (sz) 
        _pipe.put(buf, sz);
    // now parse it
    return _getMessage(&_pipe, buf, len);   
}

int GnssI2C::send(const char* buf, int len)
{
    int sent = 0;
    if (len) 
    {
        if (!I2C::write(_i2cAdr,&REGSTREAM,sizeof(REGSTREAM),true))
            sent = send(buf, len);
        stop();
    }
    return sent;
}

int GnssI2C::sendNmea(const char* buf, int len)
{ 
    int sent = 0;
    if (!I2C::write(_i2cAdr,&REGSTREAM,sizeof(REGSTREAM),true))
        sent = GnssParser::sendNmea(buf, len);
    stop();
    return sent;
}

int GnssI2C::sendUbx(unsigned char cls, unsigned char id, const void* buf, int len)
{ 
    int sent = 0;
    if (!I2C::write(_i2cAdr,&REGSTREAM,sizeof(REGSTREAM),true))
        sent = GnssParser::sendUbx(cls, id, buf, len);
    I2C::stop();
    return sent;
}

int GnssI2C::_get(char* buf, int len)
{
    int read = 0;
    unsigned char sz[2] = {0,0};
    if (!I2C::write(_i2cAdr,&REGLEN,sizeof(REGLEN),true) && 
        !I2C::read(_i2cAdr,(char*)sz,sizeof(sz)))
    {
        int size = 256 * (int)sz[0] + sz[1];
        if (size > len)
            size = len;
        if (size > 0) 
        {
            if (!I2C::write(_i2cAdr,&REGSTREAM,sizeof(REGSTREAM),true) &&
                !I2C::read(_i2cAdr,buf,size)) {
                read = size;
            }
        }
    }
    return read;
}

int GnssI2C::_send(const void* buf, int len)
{ 
    return !I2C::write(_i2cAdr,(const char*)buf,len,true) ? len : 0; 
}

const char GnssI2C::REGLEN    = 0xFD;
const char GnssI2C::REGSTREAM = 0xFF;

// End Of File
