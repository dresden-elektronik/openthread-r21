/**
 * Portable Linked List Buffer template implementation
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
#define _LINKED_LIST_BUFFER_TEMPLATE_HEADER_
#include <stddef.h>
#include <stdbool.h>

#ifndef LINKED_LIST_ASSERT
    #define LINKED_LIST_ASSERT(...) //stub
#endif

#ifdef LINKED_LIST_CONFIG_HEADER_INCLUDE
    #include LINKED_LIST_CONFIG_HEADER_INCLUDE
#endif

#ifndef LINKED_LIST_BUFFER_ENTRY_DATA_TYPE
    #define LINKED_LIST_BUFFER_ENTRY_DATA_TYPE unsigned char
#endif
#ifndef LINKED_LIST_BUFFER_ARITHMETIC_DATA_TYPE
    #define LINKED_LIST_BUFFER_ARITHMETIC_DATA_TYPE unsigned int
#endif

typedef LINKED_LIST_BUFFER_ENTRY_DATA_TYPE linkedListBufferEntryDataType_t;
typedef LINKED_LIST_BUFFER_ARITHMETIC_DATA_TYPE linkedListArithmeticDataType_t;

#ifndef _LINKED_LIST_ENTRY_STRUCT_DEFINED
#define _LINKED_LIST_ENTRY_STRUCT_DEFINED

typedef struct linkedListBufferEntry_s
{
    linkedListBufferEntryDataType_t     data;
    struct linkedListBufferEntry_s *    next;

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
    struct linkedListBufferEntry_s *    previous;
#endif

}linkedListBufferEntry_t;

#endif //_LINKED_LIST_ENTRY_STRUCT_DEFINED


#ifndef _LINKED_LIST_INSTANCE_STRUCT_DEFINED
#define _LINKED_LIST_INSTANCE_STRUCT_DEFINED

typedef struct linkedListBuffer_s
{
    linkedListBufferEntry_t *           root;
    linkedListBufferEntry_t *           bufferPool;
    const linkedListArithmeticDataType_t    bufferPoolSize;
#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
    linkedListArithmeticDataType_t          numActiveEntries;
#endif
}linkedListBuffer_t;

#endif //_LINKED_LIST_INSTANCE_STRUCT_DEFINED



#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
linkedListArithmeticDataType_t linkedList_getNumActiveBuffer(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
    return handle->numActiveEntries;
#else
    linkedListArithmeticDataType_t numActive = 0;
    linkedListBufferEntry_t * entry = handle->root;
    while (numActive < handle->bufferPoolSize)
    {
        if(entry->next == NULL)
        {
            //Invalid Buffer
            break;
        }
        numActive++;
        if(entry->next == entry)
        {
            //Last Buffer
            break;
        }
        entry = entry->next;
    }
    return numActive;
#endif
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
linkedListArithmeticDataType_t linkedList_getNumBufferLeft(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    return handle->bufferPoolSize - linkedList_getNumActiveBuffer(handle);
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
linkedListBufferEntry_t * linkedList_getLast(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    linkedListBufferEntry_t *  entry = handle->root;
    for (linkedListArithmeticDataType_t i = 0; i < handle->bufferPoolSize; i++)
    {
        if( ( entry->next == entry ) || ( entry->next == NULL ) )
        {
            return entry;
        }
        entry = entry->next;
    }
    return NULL; 
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
linkedListBufferEntry_t * linkedList_getFirst(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);

    if(handle->root->next == NULL)
    {
        return NULL;
    }
    return handle->root;
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
linkedListBufferEntry_t * linkedList_getPrevious(linkedListBuffer_t * handle, linkedListBufferEntry_t * entry)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(entry != NULL);
    LINKED_LIST_ASSERT(handle != NULL);

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED

    (void) handle;

    if( (entry == NULL) || (entry->previous == entry) || (entry->previous == NULL) )
    {
        return NULL;
    }
    return entry->previous;
#else
    
    linkedListBufferEntry_t *  previous = handle->root;

    if(entry == handle->root)
    {
        return NULL;
    }

    for (linkedListArithmeticDataType_t i = 0; i < handle->bufferPoolSize; i++)
    {
        if  (previous->next == entry )
        {
            return previous;
        }

        if(previous->next == previous)
        {
            //reached end of linked List
            break;
        }
        previous = previous->next;
    }
    //entry not part of linked List
    return NULL; 
#endif    
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
linkedListBufferEntry_t * linkedList_getNext(linkedListBuffer_t * handle, linkedListBufferEntry_t * entry)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(entry != NULL);
    LINKED_LIST_ASSERT(handle != NULL);

    (void) handle;


    if( (entry == NULL) || (entry->next == entry) || (entry->next == NULL) )
    {
        return NULL;
    }
    return entry->next;
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
void linkedList_reset(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);

    for(linkedListArithmeticDataType_t i = 0; i < handle->bufferPoolSize; i++)
    {
        handle->bufferPool[i].next = NULL;
#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
        handle->bufferPool[i].previous = NULL;
#endif
    }
    handle->root = &handle->bufferPool[0];
    handle->root->next = NULL;
#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
    handle->root->previous = NULL;
#endif

#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
    handle->numActiveEntries = 0;
#endif
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)




#ifdef LINKED_LIST_BUFFER_CLEAR_MEMORY_ON_ALLOC
#include <string.h>
#endif
#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
linkedListBufferEntry_t * linkedList_allocEntry(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);

    for (linkedListArithmeticDataType_t i = 0; i < handle->bufferPoolSize; i++)
    {
        //Find unused Buffer
        if(handle->bufferPool[i].next == NULL)
        {

            //Mark as new End
            handle->bufferPool[i].next = &handle->bufferPool[i];

            if( (handle->root == NULL) || (handle->root->next == NULL) )
            {
                //No Valid root, become root

#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
            LINKED_LIST_ASSERT(handle->numActiveEntries == 0);
#endif 

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
                handle->bufferPool[i].previous = &handle->bufferPool[i];
#endif
                handle->root = &handle->bufferPool[i];


#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
                handle->numActiveEntries = 0; //will be incremented at end of func
#endif

            }
            else
            {
                //Edit previously last Entry
                linkedListBufferEntry_t * previousEnd = linkedList_getLast(handle);
                previousEnd->next = &handle->bufferPool[i];

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED               
                handle->bufferPool[i].previous = previousEnd;
#endif
            }

#ifdef LINKED_LIST_BUFFER_CLEAR_MEMORY_ON_ALLOC
            //Reset fresh Entry
            memset(&handle->bufferPool[i].data,0x00,sizeof(linkedListBufferEntryDataType_t));
#endif

#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
            handle->numActiveEntries++;
#endif

            return &handle->bufferPool[i];
        }
    }
    //None Available
    return NULL;
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
bool linkedList_freeEntry(linkedListBuffer_t * handle, linkedListBufferEntry_t * entry)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);
    LINKED_LIST_ASSERT(entry != NULL);

#ifndef LINKED_LIST_BUFFER_DUALLY_LINKED    
    linkedListBufferEntry_t * previous = linkedList_getPrevious(handle, entry);
#endif

    if(entry == handle->root)
    {
        //the current root entry will be deleted, so the next entry becomes the new root
        //for a single entry this should not move the root, cause next is the root itself

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED    
        entry->next->previous = entry->next;
#endif
        handle->root = entry->next;
    }
    else if(entry->next == entry)
    {
        //this is the last Entry in the buffer

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED    
        entry->previous->next = entry->previous;
#else
        previous->next = previous;
#endif
    }
#ifndef LINKED_LIST_BUFFER_DUALLY_LINKED    
    else if(previous == NULL)
    {
        //Given Buffer not a part of active linked List
        LINKED_LIST_ASSERT(previous != NULL);
        return false;
    }
#endif
    else
    {
        //Entry in the middle of the linked list
#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED    
        entry->previous->next = entry->next;
        entry->next->previous = entry->previous;
#else
        previous->next = entry->next;
#endif
    }

    //invalidate the handle
    entry->next = NULL;

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED    
    entry->previous = NULL;
#endif

#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
    handle->numActiveEntries--;
#endif

    return true;
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)


#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
linkedListBufferEntry_t * linkedList_findBaseEntry(linkedListBuffer_t * handle, void * ptr)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(ptr != NULL);
    LINKED_LIST_ASSERT(handle != NULL);

  if((linkedListArithmeticDataType_t)(ptr) < (linkedListArithmeticDataType_t)(&handle->bufferPool[0]))
  {
      return NULL;
  }

  for(linkedListArithmeticDataType_t i = 1; i <= handle->bufferPoolSize; i++)
  {
      if((linkedListArithmeticDataType_t)(ptr) < (linkedListArithmeticDataType_t)(&handle->bufferPool[i]))
      {
          return &handle->bufferPool[i-1];
      }
  }
  return NULL;
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



