/* mbed TM1640 Library, for TM1640 LED controller
 * Copyright (c) 2016, v01: WH, Initial version
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, inclumosig without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUmosiG BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef TM1640_H
#define TM1640_H

// Select one of the testboards for TM1640 LED controller
#include "TM1640_Config.h"

/** An interface for driving TM1640 LED controller
 *
 * @code
 * #include "mbed.h"
 * #include "TM1640.h" 
 * 
 * DisplayData_t size is 16 bytes (16 grids @ 8 segments)
 * TM1640::DisplayData_t all_str  = {0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF, 0xFF,0xFF};  
 *
  * // TM1640 declaration
 * TM1640 TM1640(p5,p7);
 *
 * int main() {
 *   TM1640.cls(); 
 *   TM1640.writeData(all_str);
 *   wait(1);
 *   TM1640.setBrightness(TM1640_BRT0);
 *   wait(1);
 *   TM1640.setBrightness(TM1640_BRT3);
 *
 *   while (1) {
 *     TM1640.cls(); 
 *     wait(0.5); 
 *     TM1640.writeData(all_str);
 *     wait(0.5); 
 *   }  
 * }
 * @endcode
 */


//TM1640 Display data
#define TM1640_MAX_NR_GRIDS   16
#define TM1640_BYTES_PER_GRID  1

//Memory size in bytes for Display
#define TM1640_DISPLAY_MEM  (TM1640_MAX_NR_GRIDS * TM1640_BYTES_PER_GRID)

//Reserved bits for commands
#define TM1640_CMD_MSK      0xC0

//Data setting commands
#define TM1640_DATA_SET_CMD 0x40
#define TM1640_DATA_WR      0x00
#define TM1640_ADDR_INC     0x00
#define TM1640_ADDR_FIXED   0x04
#define TM1640_MODE_NORM    0x00
#define TM1640_MODE_TEST    0x08

//Address setting commands
#define TM1640_ADDR_SET_CMD 0xC0
#define TM1640_ADDR_MSK     0x0F

//Display control commands
#define TM1640_DSP_CTRL_CMD 0x80
#define TM1640_BRT_MSK      0x07
#define TM1640_BRT0         0x00 //Pulsewidth 1/16
#define TM1640_BRT1         0x01
#define TM1640_BRT2         0x02
#define TM1640_BRT3         0x03
#define TM1640_BRT4         0x04
#define TM1640_BRT5         0x05
#define TM1640_BRT6         0x06
#define TM1640_BRT7         0x07 //Pulsewidth 14/16

#define TM1640_BRT_DEF      TM1640_BRT3

#define TM1640_DSP_OFF      0x00
#define TM1640_DSP_ON       0x08


/** A class for driving TM1640 LED controller
 *
 * @brief Supports 16 Grids @ 8 Segments. 
 *        Serial bus interface device. 
 */
class TM1640 {
 public:

  /** Datatype for displaydata */
  typedef char DisplayData_t[TM1640_DISPLAY_MEM];
    
 /** Constructor for class for driving TM1640 LED controller
  *
  * @brief Supports 16 Grids @ 8 segments. 
  *        Serial bus interface device. 
  *
  *  @param  PinName mosi Serial bus MOSI pin
  *  @param  PinName sclk Serial bus SCLK pin  
  */
  TM1640(PinName mosi, PinName sclk);
      
  /** Clear the screen and locate to 0
   */ 
  void cls();  

  /** Write databyte to TM1640
   *  @param  char data byte written at given address
   *  @param  int address display memory location to write byte
   *  @return none
   */ 
   void writeData(char data, int address); 

   /** Write Display datablock to TM1640
    *  @param  DisplayData_t data Array of TM1640_DISPLAY_MEM (=16) bytes for displaydata
    *  @param  length number bytes to write (valid range 0..(TM1640_MAX_NR_GRIDS * TM1640_BYTES_PER_GRID) (=16), when starting at address 0)  
    *  @param  int address display memory location to write bytes (default = 0) 
    *  @return none
    */ 
    void writeData(DisplayData_t data, int length = (TM1640_MAX_NR_GRIDS * TM1640_BYTES_PER_GRID), int address = 0);
 
  /** Set Brightness
    *
    * @param  char brightness (3 significant bits, valid range 0..7 (1/16 .. 14/14 dutycycle)  
    * @return none
    */
  void setBrightness(char brightness = TM1640_BRT_DEF);
  
  /** Set the Display mode On/off
    *
    * @param bool display mode
    */
  void setDisplay(bool on);
  
  private:  
    DigitalOut _mosi;
    DigitalOut _sclk;  
    char _display;
    char _bright; 
  
  /** Init the SPI interface and the controller
    * @param  none
    * @return none
    */ 
    void _init();


  /** Generate Start condition for TM1640
    *  @param  none
    *  @return none
    */ 
    void _start();
  
  /** Generate Stop condition for TM1640
    *  @param  none
    *  @return none
    */ 
    void _stop();

  /** Send byte to TM1640
    *  @param  int data
    *  @return none
    */ 
    void _write(int data);

  /** Write command and parameter to TM1640
    *  @param  int cmd Command byte
    *  &Param  int data Parameters for command
    *  @return none
    */ 
    void _writeCmd(int cmd, int data);  
};


#if (LM1640_TEST == 1) 
// Derived class for TM1640 used in LM1640 display unit
//

#include "Font_7Seg.h"

#define LM1640_NR_GRIDS  16
#define LM1640_NR_DIGITS 16
#define LM1640_NR_UDC    8

/** Constructor for class for driving TM1640 controller as used in LM1640
  *
  *  @brief Supports 16 Digits of 7 Segments + DP.
  *  
  *  @param  PinName mosi Serial bus MOSI pin
  *  @param  PinName sclk Serial bus SCLK pin 
  */
class TM1640_LM1640 : public TM1640, public Stream {
 public:

  /** Enums for Icons */
  //  Grid encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
  enum Icon {
    DP1   = ( 1<<24) | S7_DP1,  /**<  Decimal Point 1 */
    DP2   = ( 2<<24) | S7_DP2,  /**<  Decimal Point 2 */
    DP3   = ( 3<<24) | S7_DP3,  /**<  Decimal Point 3 */
    DP4   = ( 4<<24) | S7_DP4,  /**<  Decimal Point 4 */
    DP5   = ( 5<<24) | S7_DP5,  /**<  Decimal Point 5 */
    DP6   = ( 6<<24) | S7_DP6,  /**<  Decimal Point 6 */
    DP7   = ( 7<<24) | S7_DP7,  /**<  Decimal Point 7 */
    DP8   = ( 8<<24) | S7_DP8,  /**<  Decimal Point 8 */   
    DP9   = ( 9<<24) | S7_DP9,  /**<  Decimal Point 9 */
    DP10  = (10<<24) | S7_DP10, /**<  Decimal Point 10 */
    DP11  = (11<<24) | S7_DP11, /**<  Decimal Point 11 */
    DP12  = (12<<24) | S7_DP12, /**<  Decimal Point 12 */
    DP13  = (13<<24) | S7_DP13, /**<  Decimal Point 13 */
    DP14  = (14<<24) | S7_DP14, /**<  Decimal Point 14 */
    DP15  = (15<<24) | S7_DP15, /**<  Decimal Point 15 */
    DP16  = (16<<24) | S7_DP16  /**<  Decimal Point 16 */  

  };
  
  typedef char UDCData_t[LM1640_NR_UDC];
  
 /** Constructor for class for driving TM1640 LED controller as used in LM1640
   *
   * @brief Supports 16 Digits of 7 Segments + DP.
   *  
   *  @param  PinName mosi Serial bus MOSI pin
   *  @param  PinName sclk Serial bus SCLK pin   
   */
  TM1640_LM1640(PinName mosi, PinName sclk);

#if DOXYGEN_ONLY
    /** Write a character to the Display
     *
     * @param c The character to write to the display
     */
    int putc(int c);

    /** Write a formatted string to the Display
     *
     * @param format A printf-style format string, followed by the
     *               variables to use in formatting the string.
     */
    int printf(const char* format, ...);   
#endif

     /** Locate cursor to a screen column
     *
     * @param column  The horizontal position from the left, indexed from 0
     */
    void locate(int column);
    
    /** Clear the screen and locate to 0
     * @param bool clrAll Clear Icons also (default = false)
     */
    void cls(bool clrAll = false);

    /** Set Icon
     *
     * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
     * @return none
     */
    void setIcon(Icon icon);

    /** Clr Icon
     *
     * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
     * @return none
     */
    void clrIcon(Icon icon);

   /** Set User Defined Characters (UDC)
     *
     * @param unsigned char udc_idx   The Index of the UDC (0..7)
     * @param int udc_data            The bitpattern for the UDC (16 bits)       
     */
    void setUDC(unsigned char udc_idx, int udc_data);


   /** Number of screen columns
    *
    * @param none
    * @return columns
    */
    int columns();   

   /** Write databyte to TM1640
     *  @param  char data byte written at given address
     *  @param  int address display memory location to write byte
     *  @return none
     */ 
    void writeData(char data, int address){
      TM1640::writeData(data, address);
    }        

   /** Write Display datablock to TM1640
    *  @param  DisplayData_t data Array of TM1640_DISPLAY_MEM (=16) bytes for displaydata
    *  @param  length number bytes to write (valid range 0..(LM1640_NR_GRIDS * TM1640_BYTES_PER_GRID) (=16), when starting at address 0)  
    *  @param  int address display memory location to write bytes (default = 0) 
    *  @return none
    */   
    void writeData(DisplayData_t data, int length = (LM1640_NR_GRIDS * TM1640_BYTES_PER_GRID), int address = 0) {
      TM1640::writeData(data, length, address);
    }  

protected:  
    // Stream implementation functions
    virtual int _putc(int value);
    virtual int _getc();

private:
    int _column;
    int _columns;   
    
    DisplayData_t _displaybuffer;
    UDCData_t _UDC_7S; 
};
#endif

#endif