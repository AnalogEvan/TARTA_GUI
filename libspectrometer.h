/** \file
 * \defgroup API libspectrometer API
 */

#ifndef LIBSHARED_AND_STATIC_EXPORT_H
#define LIBSHARED_AND_STATIC_EXPORT_H

#if defined (_WIN32)
    #ifndef LIBSHARED_AND_STATIC_EXPORT
        #if !defined(AS_CORE_LIBRARY)
            #ifdef libspectrometer_EXPORTS
                #define LIBSHARED_AND_STATIC_EXPORT __declspec(dllexport)
            #else
                #define LIBSHARED_AND_STATIC_EXPORT __declspec(dllimport)
            #endif
        #else
            #define LIBSHARED_AND_STATIC_EXPORT
        #endif       
    #endif
  
    #ifndef LIBSHARED_AND_STATIC_DEPRECATED
        #define LIBSHARED_AND_STATIC_DEPRECATED __declspec(deprecated)
        #define LIBSHARED_AND_STATIC_DEPRECATED_EXPORT LIBSHARED_AND_STATIC_EXPORT __declspec(deprecated)
        #define LIBSHARED_AND_STATIC_DEPRECATED_NO_EXPORT LIBSHARED_AND_STATIC_NO_EXPORT __declspec(deprecated)
    #endif
#else
    #define LIBSHARED_AND_STATIC_EXPORT
#endif /* defined (_WIN32) */

#ifndef AS_CORE_LIBRARY
    #ifdef __cplusplus
        extern "C" {
    #endif
#endif

#include <stdint.h>

typedef unsigned char uint8_t;
typedef unsigned short uint16_t;
typedef unsigned int uint32_t;

#ifndef DEVICE_INFO
#define DEVICE_INFO
typedef struct DeviceInfo_t{
      char* serialNumber;
      struct DeviceInfo_t *next;
} DeviceInfo_t;
#endif


/** \brief Free a device handle 
    
    This function frees a device handle initialized by connectToDeviceBySerial() or connectToDeviceByIndex()
    
    \param[in] deviceContextPtr
    \parblock
    Provide the address of a valid uintptr_t variable (containing a handle previously initialized by connectToDeviceBySerial() or connectToDeviceByIndex())
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int disconnectDeviceContext(uintptr_t* deviceContextPtr);

/** \brief Finds the requested device by the provided serial number and connects to it.
    This function or its analog connectToDeviceByIndex() should always precede all other library operations.

    \param[in] serialNumber
    \parblock
    If the serialNumber pointer is NULL, the function will connect to a first found device with VID = 0xE220 and PID = 0x0100
    \endparblock

    \param[out] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable.
    The handle inside will be initialized with device state information required for all the other functions.
    Set the uintptr_t variable to 0 before calling this function for the first time to start working with a device. 
    If the requested device is already connected (dereferenced pointer value does not equal 0), disconnects the device and reinitializes the handle with a new device state. 
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int connectToDeviceBySerial(const char * const serialNumber, uintptr_t* deviceContextPtr);

/** \brief Connects to the required device by index. 

    This function or connectToDeviceBySerial() should always precede all other library operations.

    \param[in] index
    \parblock
    The value of the index parameter can vary from 0 to n-1 (where n is the number of the connected devices)
    If only one device is connected use 0 as the index parameter.
    \endparblock    

    \param[out] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable.
    The handle inside will be initialized with device state information required for all the other functions.
    Set the uintptr_t variable to 0 before calling this function for the first time to start working with a device. 
    If the requested device is already connected (dereferenced pointer value does not equal 0), disconnects the device and reinitializes the handle with a new device state. 
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int connectToDeviceByIndex(unsigned int index, uintptr_t *deviceContextPtr);

/* Deprecated - left for internal use only
LIBSHARED_AND_STATIC_EXPORT void disconnectDevice();
*/

/** \brief Returns the number of the connected devices

    \ingroup API

    \returns This function returns the number of connected devices with VID = 0xE220 and PID = 0x0100
*/
LIBSHARED_AND_STATIC_EXPORT uint32_t getDevicesCount();

/** \brief Obtains the list of all connected devices.
Run clearDevicesInfo() with the result of this function to clear it [Mandatory]
\details {
structure information:
struct DeviceInfo_t {
      char* serial; //descriptor to the string with serial number
      struct DeviceInfo_t *next; //descriptor to the next element of the list or null if last element
}
}

\ingroup API

\returns the list of all connected devices (afterwards should be freed with clearDevicesInfo() function!)
*/
LIBSHARED_AND_STATIC_EXPORT DeviceInfo_t * getDevicesInfo();

/** \brief Clears the list of all connected devices obtained by getDevicesInfo() function

    \note This function must be used to clear the result of the getDevicesInfo() function, otherwise there will be a memory leak

    \ingroup API
*/
LIBSHARED_AND_STATIC_EXPORT void clearDevicesInfo(DeviceInfo_t *devices);

/** \brief Sets frame parameters
\note this function clears the memory and stops the current acquisition

\param[in] numOfStartElement
\parblock
Minimum value is 0
Maximum value is 3647

if numOfStartElement == numOfEndElement, a frame will contain (32 starting elements) + (1 user element) + (14 final elements).
\endparblock

\param[in] numOfEndElement
\parblock
Minimum value is 0
Maximum value is 3647

if numOfStartElement == numOfEndElement, a frame will contain (32 starting elements) + (1 user element) + (14 final elements).
\endparblock

\param[in] reductionMode
\parblock
reductionMode defines the mode of pixels averaging
    0 - no average
    1 - average of 2
    2 - average of 4
    3 - average of 8
\endparblock

\param[out] numOfPixelsInFrame
The numOfPixelsInFrame parameter is used to return the frame size in pixels. Frame size in bytes = (2 * numOfPixelsInFrame)
Use NULL to exclude this parameter.
\parblock
\endparblock

\param[out] deviceContextPtr
\parblock
This pointer should not be NULL - provide the address of a valid uintptr_t variable
(The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
\endparblock

\ingroup API

\returns   This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int setFrameFormat(uint16_t numOfStartElement, uint16_t numOfEndElement, uint8_t reductionMode, uint16_t *numOfPixelsInFrame, uintptr_t *deviceContextPtr);

/** \brief Sets exposure parameter

Can be called in the middle of acquisition, exposure will be applied on the next frame

\param[in] timeOfExposure = multiple of 10 us (microseconds)

\param[in] force - set force

\param[in] deviceContextPtr
\parblock
This pointer should not be NULL - provide the address of a valid uintptr_t variable
(The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
\endparblock

\ingroup API

\returns This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int setExposure(uint32_t timeOfExposure, uint8_t force, uintptr_t *deviceContextPtr);

/** \brief Sets acquisition parameters
    \note current acquisition will be stopped by calling this function

    \param[in] numOfScans - up to 137 full spectra
    \param[in] numOfBlankScans - check the description of the scanMode parameter below
    \param[in] scanMode
    \parblock
    0 - continuous scanMode

    CCD is read continuously. On trigger numOfScans frames will be stored in memory, with numOfBlankScans blank frames between them.

    1 - first frame idle scan mode

    no activity on CCD before trigger. On trigger numOfScans frames with numOfBlankScans blank frames between them will be read automaticly, then CCD enters idle mode until the next trigger.

    2 - every frame idle scan mode

    no activity on CCD before trigger. Every frame, including blank frames will be read only on trigger.

    3 - frame averaging mode. CCD is read continuously. Every numOfScans frames are averaged, result can be loaded with getFrame(0xFFFF) function. numOfBlankScans in this mode does not make sense and has to be set to 0
    \endparblock
    \param[in] timeOfExposure multiple of 10 us (microseconds)

    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
(The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)

    \ingroup API

\returns
    This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int setAcquisitionParameters(uint16_t numOfScans, uint16_t numOfBlankScans, uint8_t scanMode, uint32_t timeOfExposure, uintptr_t *deviceContextPtr);

/** \brief Sets multiple parameters - combination of setAcquisitionParameters() and setExternalTrigger() functions
    Clears memory and sets the required parameters (the parameters for the setAcquisitionParameters() and setExternalTrigger() functions). If enableMode = 0 or signalFrontMode = 0 (external trigger disabled) this function also triggers the acquisition
    \param[in] numOfScans - up to 137 full spectra
    \param[in] numOfBlankScans - check the description of the scanMode parameter below
    \param[in] scanMode
    \parblock
    0 - continuous scanMode

    CCD is read continuously. On trigger numOfScans frames will be stored in memory, with numOfBlankScans blank frames between them.

    1 - first frame idle scan mode

    no activity on CCD before trigger. On trigger numOfScans frames with numOfBlankScans blank frames between them will be read automaticly, then CCD enters idle mode until the next trigger.

    2 - every frame idle scan mode

    no activity on CCD before trigger. Every frame, including blank frames will be read only on trigger.

    3 - frame averaging mode. CCD is read continuously. Every numOfScans frames are averaged, result can be loaded with getFrame(0xFFFF) function. numOfBlankScans in this mode does not make sense and has to be set to 0
    \endparblock
    \param[in] timeOfExposure multiple of 10 us (microseconds)
    \param[in] enableMode
    \parblock
    enableMode:
    0 - trigger disabled
    1 - trigger enabled
    2 - one time trigger
    \endparblock
    \param[in] signalFrontMode
    \parblock
    triggerFront:
    0 - trigger disabled
    1 - front rising
    2 - front falling
    3 - both rising and fall
    \endparblock        
    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int setMultipleParameters(uint16_t numOfScans, uint16_t numOfBlankScans, uint8_t scanMode, uint32_t timeOfExposure, uint8_t enableMode, uint8_t signalFrontMode, uintptr_t *deviceContextPtr);

/** \brief Sets the external acquisition trigger
    \param[in] enableMode
    \parblock
    enableMode:
    0 - trigger disabled
    1 - trigger enabled
    2 - one time trigger
    \endparblock
    \param[in] signalFrontMode
    \parblock
    triggerFront:
    0 - trigger disabled
    1 - front rising
    2 - front falling
    3 - both rising and fall
    \endparblock
    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int setExternalTrigger(uint8_t enableMode, uint8_t signalFrontMode, uintptr_t *deviceContextPtr);

/** \brief Sets the optical trigger
    \note Only applicable in scanMode = 0 (check the acquisition parameters)

    \param[in] enableMode
    \parblock
    enableMode = 0 trigger disabled
    enableMode = 1 trigger for falling edge
    enableMode = 2 trigger on threshold
    enableMode = 0x81 one time trigget for rising edge
    enableMode = 0x82 one time trigger for falling edge
    \endparblock
    \param[in] pixel
    \parblock
    Pixel number (for 0 to 3639 range) for with exiding level should be detected (for optical trigger only). Does not depend on frame format.
    \endparblock
    \param[in] threshold

    \param[out] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int setOpticalTrigger(uint8_t enableMode, uint16_t pixel, uint16_t threshold, uintptr_t *deviceContextPtr);

/** \brief Start acquisition by software    
    \param[in] deviceContextPtr
    \parblock
    If the threadSafeMode is set to true, this pointer provides the device state information required for all the other functions.
    (Initialize this pointer with any of the connectToSingleDevice* functions and provide it to the other functions in this library to work in the thread-safe mode)
    If the requested device is already connected, disconnects and initializes the state again(in thread-safe mode)
    \endparblock

    \internal
    outReport[1]=6;
    \endinternal

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.

*/
LIBSHARED_AND_STATIC_EXPORT int triggerAcquisition(uintptr_t *deviceContextPtr);

/** \brief Gets the device status.
    \param[out] statusFlags - provide an initialized pointer or NULL to skip this parameter
    \parblock
    statusFlags & 1 - operation in progress

    you can read already stored frames while operation in progress if framesInMemory > frame number to read.

    statusFlags & 2 - memory is full

    on this flag acquisition will be stopped and new acquisition can't be started until memory is cleared by clearMemory() or setFrameFormat()
    \endparblock
    \param[out] framesInMemory - provide an initialized pointer or NULL to skip this parameter
    \parblock
    framesInMemory returns current number of frames in memory.
    for frame averaging mode:
        framesInMemory = 0 - spectra is not ready 
        framesInMemory = 1 - spectra is ready for read
        framesInMemory = 2 - spectra is ready and at least one spectra is lost because it's not been read.
    \endparblock
    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int getStatus(uint8_t *statusFlags, uint16_t *framesInMemory,  uintptr_t *deviceContextPtr);

/** \brief Returns the same values as set by setAcquisitionParameters
    \param[out] numOfScans - provide an initialized pointer or NULL to skip this parameter fetch
    \param[out] numOfBlankScans - provide an initialized pointer or NULL to skip this parameter fetch
    \param[out] scanMode - provide an initialized pointer or NULL to skip this parameter fetch
    \param[out] timeOfExposure - provide an initialized pointer or NULL to skip this parameter fetch

    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int getAcquisitionParameters(uint16_t* numOfScans, uint16_t* numOfBlankScans, uint8_t *scanMode, uint32_t* timeOfExposure, uintptr_t *deviceContextPtr);

/** \brief Returns the same values as set by setFrameFormat
    \param[out] numOfStartElement
    \param[out] numOfEndElement
    \param[out] reductionMode
    \param[out] numOfPixelsInFrame

    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int getFrameFormat(uint16_t *numOfStartElement, uint16_t *numOfEndElement, uint8_t *reductionMode, uint16_t *numOfPixelsInFrame, uintptr_t *deviceContextPtr);

/** \brief Gets frame
    \param[out] framePixelsBuffer - provide an initialized pointer to the buffer of unsigned short elements.
    \param[in] numOfFrame
    \parblock
    numOfFrame - first frame in memory is number 0, second is 1, etc
    numOfFrame = 0xFFFF - calculate averaged spectrum (for averaging mode), for all other modes function will return last captured frame
    \endparblock

    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int getFrame(uint16_t  *framePixelsBuffer, uint16_t numOfFrame, uintptr_t *deviceContextPtr);

/** \brief Clears memory

    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int clearMemory(uintptr_t *deviceContextPtr);

/** \brief Erases user flash memory.
    \note There is no way to partially erase it.

    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int eraseFlash(uintptr_t *deviceContextPtr);

/** \brief Reads from user flash memory
     Reads bytesToRead from user flash memory starting at offset and copies them to the buffer
    \param[out] buffer - provide an initialized pointer to the buffer of unsigned char elements.
    \param[in] absoluteOffset
    \parblock
    Any value from 0 to 1FFFF*
    *you can read only 1 byte from 1FFFF, 2 bytes from 1FFFE etc.
    \endparblock
    \param[in] bytesToRead

    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int readFlash(uint8_t *buffer, uint32_t absoluteOffset, uint32_t bytesToRead, uintptr_t *deviceContextPtr);

/** \brief Writes bytesToWrite bytes from the buffer to the user flash memory starting at offset
    \param[out] buffer
    \param[in] absoluteOffset
    \parblock
    offset can be any value from 0 to 1FFFF*
    *you can write only 1 byte to 1FFFF, 2 bytes to 1FFFE etc.

    you can write only to empty memory locations that have values = 0xff.
    The only way to get their state back to empty is erasing all user memory by eraseFlash(). There is no way to partially erase it.
    128kb available
    \endparblock
    \param[in] bytesToWrite

    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int writeFlash(uint8_t *buffer, uint32_t absoluteOffset, uint32_t bytesToWrite,  uintptr_t *deviceContextPtr);

/** \brief Resets all the device parameters to their default values and clears the memory
    \param[in] deviceContextPtr
    \parblock
    This pointer should not be NULL - provide the address of a valid uintptr_t variable
    (The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
    \endparblock

    \note
    default parameters:
    numOfStartElement = 0
    numOfEndElement = 3647
    reductionMode = 0
    numOfScans = 1
    numOfBlankScans = 0
    scanMode = 0
    exposureTime = 10


    \ingroup API

    \returns
        This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int resetDevice(uintptr_t *deviceContextPtr);

/** \brief Disconnects the device
The device will be disconnected from USB, and any interaction with it will not be possible until the device is reset

\param[in] deviceContextPtr
\parblock
This pointer should not be NULL - provide the address of a valid uintptr_t variable
(The uintptr_t variable contains the device state information handle and should be previously initialized by either connectToDeviceBySerial() or connectToDeviceByIndex() function)
\endparblock

\ingroup API

\returns
This function returns 0 on success and error code in case of error.
*/
LIBSHARED_AND_STATIC_EXPORT int detachDevice(uintptr_t *deviceContextPtr);

/**   \ingroup API */
#ifndef SPECTROMETER_ERROR_CODES
#define SPECTROMETER_ERROR_CODES
    /** \ingroup API */
    #define OK 0
    /** \ingroup API */
    #define CONNECT_ERROR_WRONG_ID 500
    /** \ingroup API */
    #define CONNECT_ERROR_NOT_FOUND 501
    /** \ingroup API */
    #define CONNECT_ERROR_FAILED 502
    /** \ingroup API */
    #define DEVICE_NOT_INITIALIZED 503
    /** \ingroup API */
    #define WRITING_PROCESS_FAILED 504
    /** \ingroup API */
    #define READING_PROCESS_FAILED 505
    /** \ingroup API */
    #define WRONG_ANSWER 506
    /** \ingroup API */
    #define GET_FRAME_REMAINING_PACKETS_ERROR 507
    /** \ingroup API */
    #define NUM_OF_PACKETS_IN_FRAME_ERROR 508
    /** \ingroup API */
    #define INPUT_PARAMETER_NOT_INITIALIZED 509
    /** \ingroup API */
    #define READ_FLASH_REMAINING_PACKETS_ERROR 510
    /** \ingroup API */
    #define CONNECT_ERROR_WRONG_SERIAL_NUMBER 516
    /** \ingroup API */
    #define NO_DEVICE_CONTEXT_ERROR 585
#endif

#ifndef AS_CORE_LIBRARY
    #ifdef __cplusplus
        }
    #endif
#endif

#endif
