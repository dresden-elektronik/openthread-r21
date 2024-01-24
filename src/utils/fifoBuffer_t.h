/**
 * Portable Fifo Buffer template implementation
 * 
 * Author:    Haerteleric
 * 
 * MIT License
 * 
 * Copyright (c) 2023 Eric Härtel
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 **/
#include <stddef.h>
#include <stdbool.h>

#ifndef _FIFO_BUFFER_INCLUDED
#define _FIFO_BUFFER_INCLUDED
#endif

#ifdef FIFO_BUFFER_CONFIG_HEADER_INCLUDE
    #include FIFO_BUFFER_CONFIG_HEADER_INCLUDE
#endif

#ifndef FIFO_BUFFER_ENTRY_DATA_TYPE
    #define FIFO_BUFFER_ENTRY_DATA_TYPE unsigned int
#endif

#ifndef FIFO_BUFFER_ARITHMETIC_DATA_TYPE
    #define FIFO_BUFFER_ARITHMETIC_DATA_TYPE unsigned int
#endif

typedef FIFO_BUFFER_ENTRY_DATA_TYPE fifoBufferEntryDataType_t;
typedef FIFO_BUFFER_ARITHMETIC_DATA_TYPE fifoBufferArithmeticDataType_t;


#ifndef _FIFO_BUFFER_INSTANCE_STRUCT_DEFINED
#define _FIFO_BUFFER_INSTANCE_STRUCT_DEFINED

typedef struct fifoBuffer_s
{
    fifoBufferEntryDataType_t  *buffer;
    const fifoBufferArithmeticDataType_t bufferSize;
    fifoBufferArithmeticDataType_t readHeadPos;
    fifoBufferArithmeticDataType_t writeHeadPos;
}fifoBuffer_t;

#endif // _FIFO_BUFFER_INSTANCE_STRUCT_DEFINED



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
fifoBufferArithmeticDataType_t fifo_getCurrentSize(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    if(handle->readHeadPos <= handle->writeHeadPos) 
    {
        return handle->writeHeadPos - handle->readHeadPos;
    }
    return handle->writeHeadPos + handle->bufferSize - handle->readHeadPos;
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)


#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
fifoBufferArithmeticDataType_t fifo_getFreeSize(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    return handle->bufferSize - 1 - fifo_getCurrentSize(handle);
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
fifoBufferArithmeticDataType_t fifo_getMaxSize(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    return handle->bufferSize - 1;
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
void fifo_reset(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    handle->readHeadPos = 0;
    handle->writeHeadPos = 0;
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_CLEAR_MEMORY_ON_ALLOC
#include <string.h>
#endif

#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
fifoBufferEntryDataType_t * fifo_alloc(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    fifoBufferArithmeticDataType_t nextWriteHeadPos = ( handle->writeHeadPos + 1 ) % handle->bufferSize;
    if( nextWriteHeadPos == handle->readHeadPos)
    {
        return NULL;
    }
    fifoBufferEntryDataType_t * allocedEntry = &handle->buffer[handle->writeHeadPos];

#ifdef FIFO_BUFFER_CLEAR_MEMORY_ON_ALLOC
    memset(allocedEntry, 0x00, sizeof(fifoBufferEntryDataType_t));
#endif

    handle->writeHeadPos = nextWriteHeadPos;
    return allocedEntry;
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
bool fifo_put(fifoBuffer_t * handle, fifoBufferEntryDataType_t * entry)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    fifoBufferArithmeticDataType_t nextWriteHeadPos = ( handle->writeHeadPos + 1 ) % handle->bufferSize;
    if( nextWriteHeadPos == handle->readHeadPos)
    {
        return false;
    }
    handle->buffer[handle->writeHeadPos] = *entry; 
    handle->writeHeadPos = nextWriteHeadPos;
    return true;
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)


inline static fifoBufferArithmeticDataType_t fifo_write(fifoBuffer_t * handle, fifoBufferEntryDataType_t * data, fifoBufferArithmeticDataType_t numEntries)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    fifoBufferArithmeticDataType_t nWritten = 0;
    while (nWritten < numEntries)
    {
        if(!fifo_put(handle, &data[nWritten]))
        {
            break;
        }
        nWritten++;
    }
    return nWritten;
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
fifoBufferEntryDataType_t * fifo_peakFirst(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    if( handle->writeHeadPos == handle->readHeadPos)
    {
        return NULL;
    }
    return &handle->buffer[handle->readHeadPos];
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
fifoBufferEntryDataType_t * fifo_peak(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    return fifo_peakFirst(handle);
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
fifoBufferEntryDataType_t * fifo_peakLast(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    if( handle->writeHeadPos == handle->readHeadPos)
    {
        return NULL;
    }

    if(handle->writeHeadPos == 0)
    {
        return &handle->buffer[(handle->bufferSize - 1)];
    }
    
    return &handle->buffer[(handle->writeHeadPos - 1) % handle->bufferSize];
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
void fifo_discardFirst(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    if( handle->writeHeadPos != handle->readHeadPos)
    {
        handle->readHeadPos = ( handle->readHeadPos + 1 ) % handle->bufferSize;
    }
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
void fifo_discard(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    fifo_discardFirst(handle);
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
void fifo_discardLast(fifoBuffer_t * handle)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    if( handle->writeHeadPos != handle->readHeadPos)
    {
        if(handle->writeHeadPos == 0)
        {
            handle->writeHeadPos = handle->bufferSize - 1;
        }
        handle->writeHeadPos = ( handle->writeHeadPos - 1 ) % handle->bufferSize;
    }
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
bool fifo_getFirst(fifoBuffer_t * handle, fifoBufferEntryDataType_t * buffer)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    //find pending
    fifoBufferEntryDataType_t * temp = fifo_peakFirst(handle);

    if( temp == NULL )
    {
        return false;
    }
    //Copy
    *buffer = *temp;
    
    //delete pending
    fifo_discardFirst(handle);
    return true;
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
bool fifo_get(fifoBuffer_t * handle, fifoBufferEntryDataType_t * buffer)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    return fifo_getFirst(handle, buffer);
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
bool fifo_getLast(fifoBuffer_t * handle, fifoBufferEntryDataType_t * buffer)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    //find latest
    fifoBufferEntryDataType_t * temp = fifo_peakLast(handle);

    if( temp == NULL )
    {
        return false;
    }
    //Copy
    *buffer = *temp;
    
    //delete latest
    fifo_discardLast(handle);
    return true;
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)



#ifdef FIFO_BUFFER_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef FIFO_BUFFER_STATIC_IMPLEMENTATION
static
#endif 
fifoBufferArithmeticDataType_t fifo_read(fifoBuffer_t * handle, fifoBufferEntryDataType_t * buffer, fifoBufferArithmeticDataType_t bufferSize)
#ifdef FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    fifoBufferArithmeticDataType_t nRead = 0;
    while (nRead < bufferSize)
    {
        if(!fifo_get(handle,&buffer[nRead]))
            break;
    }
    return nRead;
}
#endif // NOT(FIFO_BUFFER_ONLY_PROTOTYPE_DECLARATION)
