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
    struct linkedListBufferEntry_s *    nextEntry;

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
    struct linkedListBufferEntry_s *    previousEntry;
#endif

}linkedListBufferEntry_t;

#endif //_LINKED_LIST_ENTRY_STRUCT_DEFINED


#ifndef _LINKED_LIST_INSTANCE_STRUCT_DEFINED
#define _LINKED_LIST_INSTANCE_STRUCT_DEFINED

typedef struct linkedListBuffer_s
{
    linkedListBufferEntry_t *           rootEntry;
    linkedListBufferEntry_t *           entryPool;
    const linkedListArithmeticDataType_t    entryPoolSize;
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
linkedListArithmeticDataType_t linkedList_getNumEntriesUsed(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
    return handle->numActiveEntries;
#else
    linkedListArithmeticDataType_t numActive = 0;
    linkedListBufferEntry_t * entry = handle->rootEntry;
    while (numActive < handle->entryPoolSize)
    {
        if(entry->nextEntry == NULL)
        {
            //Invalid Buffer
            break;
        }
        numActive++;
        if(entry->nextEntry == entry)
        {
            //Last Buffer
            break;
        }
        entry = entry->nextEntry;
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
linkedListArithmeticDataType_t linkedList_getNumEntriesFree(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    return handle->entryPoolSize - linkedList_getNumEntriesUsed(handle);
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



#ifdef LINKED_LIST_INLINE_IMPLEMENTATION
inline
#endif 
#ifdef LINKED_LIST_STATIC_IMPLEMENTATION
static
#endif 
linkedListBufferEntry_t * linkedList_getLastEntry(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);

    if(!handle->rootEntry || !handle->rootEntry->nextEntry)
    {
        //No Entry in list
        return NULL;
    }

    linkedListBufferEntry_t *  entry = handle->rootEntry;
    for (linkedListArithmeticDataType_t i = 0; i < handle->entryPoolSize; i++)
    {
        if( ( entry->nextEntry == entry ) || ( entry->nextEntry == NULL ) )
        {
            return entry;
        }
        entry = entry->nextEntry;
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
linkedListBufferEntry_t * linkedList_getFirstEntry(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);

    if(!handle->rootEntry || !handle->rootEntry->nextEntry)
    {
        //No Entry in list
        return NULL;
    }

    if(handle->rootEntry->nextEntry == NULL)
    {
        return NULL;
    }
    return handle->rootEntry;
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

    if( (entry == NULL) || (entry->previousEntry == entry) || (entry->previousEntry == NULL) )
    {
        return NULL;
    }
    return entry->previousEntry;
#else
    
    linkedListBufferEntry_t *  previousEntry = handle->rootEntry;

    if(entry == handle->rootEntry)
    {
        return NULL;
    }

    for (linkedListArithmeticDataType_t i = 0; i < handle->entryPoolSize; i++)
    {
        if  (previousEntry->nextEntry == entry )
        {
            return previousEntry;
        }

        if(previousEntry->nextEntry == previousEntry)
        {
            //reached end of linked List
            break;
        }
        previousEntry = previousEntry->nextEntry;
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


    if( (entry == NULL) || (entry->nextEntry == entry) || (entry->nextEntry == NULL) )
    {
        return NULL;
    }
    return entry->nextEntry;
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

    for(linkedListArithmeticDataType_t i = 0; i < handle->entryPoolSize; i++)
    {
        handle->entryPool[i].nextEntry = NULL;
#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
        handle->entryPool[i].previousEntry = NULL;
#endif
    }
    handle->rootEntry = &handle->entryPool[0];
    handle->rootEntry->nextEntry = NULL;
#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
    handle->rootEntry->previousEntry = NULL;
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
linkedListBufferEntry_t * linkedList_allocEntryAtEnd(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);

    for (linkedListArithmeticDataType_t i = 0; i < handle->entryPoolSize; i++)
    {
        //Find unused Buffer
        if(handle->entryPool[i].nextEntry == NULL)
        {

            //Mark as new End
            handle->entryPool[i].nextEntry = &handle->entryPool[i];

            if( (handle->rootEntry == NULL) || (handle->rootEntry->nextEntry == NULL) )
            {
                //No Valid rootEntry, become rootEntry

#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
            LINKED_LIST_ASSERT(handle->numActiveEntries == 0);
#endif 

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
                handle->entryPool[i].previousEntry = &handle->entryPool[i];
#endif
                handle->rootEntry = &handle->entryPool[i];


#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
                handle->numActiveEntries = 0; //will be incremented at end of func
#endif

            }
            else
            {
                //Edit previously last Entry
                linkedListBufferEntry_t * previousEnd = linkedList_getLastEntry(handle);
                previousEnd->nextEntry = &handle->entryPool[i];

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED               
                handle->entryPool[i].previousEntry = previousEnd;
#endif
            }

#ifdef LINKED_LIST_BUFFER_CLEAR_MEMORY_ON_ALLOC
            //Reset fresh Entry
            memset(&handle->entryPool[i].data,0x00,sizeof(linkedListBufferEntryDataType_t));
#endif

#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
            handle->numActiveEntries++;
#endif

            return &handle->entryPool[i];
        }
    }
    //None Available
    return NULL;
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
linkedListBufferEntry_t * linkedList_allocEntryAtBegin(linkedListBuffer_t * handle)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);

    linkedListBufferEntry_t * previousRoot =( ((handle->rootEntry) && (handle->rootEntry->nextEntry)) ? handle->rootEntry : NULL);

    for (linkedListArithmeticDataType_t i = 0; i < handle->entryPoolSize; i++)
    {
        //Find unused Buffer
        if(handle->entryPool[i].nextEntry == NULL)
        {

            //Link previous rootEntry as next entry or mark entry as end of linked list if it was empty before
            handle->entryPool[i].nextEntry = ( previousRoot ? previousRoot : &handle->entryPool[i] );

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
            //mark as Begin of Linked List
            handle->entryPool[i].previousEntry = &handle->entryPool[i];

            
            if(previousRoot)
            {
                //Mark Root as previous of previousRoot
                previousRoot->previousEntry = &handle->entryPool[i];
            }
#endif

            //become new rootEntry
            handle->rootEntry = &handle->entryPool[i];

#ifdef LINKED_LIST_BUFFER_CLEAR_MEMORY_ON_ALLOC
            //Reset fresh Entry
            memset(&handle->entryPool[i].data,0x00,sizeof(linkedListBufferEntryDataType_t));
#endif

#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
            handle->numActiveEntries++;
#endif

            return &handle->entryPool[i];
        }
    }
    //None Available
    return NULL;
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
linkedListBufferEntry_t * linkedList_allocEntryBefore(linkedListBuffer_t * handle, linkedListBufferEntry_t * target)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);
    LINKED_LIST_ASSERT(target != NULL);

    if(target == linkedList_getFirstEntry(handle))
    {
        return linkedList_allocEntryAtBegin(handle);
    }

    linkedListBufferEntry_t * entryBeforeTarget = linkedList_getPrevious(handle,target);

    LINKED_LIST_ASSERT(entryBeforeTarget != NULL);

    for (linkedListArithmeticDataType_t i = 0; i < handle->entryPoolSize; i++)
    {
        //Find unused Buffer
        if(handle->entryPool[i].nextEntry == NULL)
        {

            //Link Target Entry as next of new Entry 
            handle->entryPool[i].nextEntry = target;

            //Mark new Entry as next Entry of Entry before Target
            entryBeforeTarget->nextEntry = &handle->entryPool[i]; 
            

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
            handle->entryPool[i].previousEntry = entryBeforeTarget;
            target->previousEntry = &handle->entryPool[i];
#endif

#ifdef LINKED_LIST_BUFFER_CLEAR_MEMORY_ON_ALLOC
            //Reset fresh Entry
            memset(&handle->entryPool[i].data,0x00,sizeof(linkedListBufferEntryDataType_t));
#endif

#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
            handle->numActiveEntries++;
#endif

            return &handle->entryPool[i];
        }
    }
    //None Available
    return NULL;
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
linkedListBufferEntry_t * linkedList_allocEntryAfter(linkedListBuffer_t * handle, linkedListBufferEntry_t * target)
#ifdef LINKED_LIST_ONLY_PROTOTYPE_DECLARATION
;
#else
{
    LINKED_LIST_ASSERT(handle != NULL);
    LINKED_LIST_ASSERT(target != NULL);

    if(target == linkedList_getLastEntry(handle))
    {
        return linkedList_allocEntryAtEnd(handle);
    }

    linkedListBufferEntry_t * entryAfterTarget = linkedList_getNext(handle,target);

    LINKED_LIST_ASSERT(entryAfterTarget != NULL);

    for (linkedListArithmeticDataType_t i = 0; i < handle->entryPoolSize; i++)
    {
        //Find unused Buffer
        if(handle->entryPool[i].nextEntry == NULL)
        {

            target->nextEntry = &handle->entryPool[i];
            handle->entryPool[i].nextEntry = entryAfterTarget;

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED
            entryAfterTarget->previousEntry = &handle->entryPool[i];
            handle->entryPool[i].previousEntry = target;
#endif

#ifdef LINKED_LIST_BUFFER_CLEAR_MEMORY_ON_ALLOC
            //Reset fresh Entry
            memset(&handle->entryPool[i].data,0x00,sizeof(linkedListBufferEntryDataType_t));
#endif

#ifdef LINKED_LIST_BUFFER_SIZE_AWARE
            handle->numActiveEntries++;
#endif

            return &handle->entryPool[i];
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
    linkedListBufferEntry_t * previousEntry = linkedList_getPrevious(handle, entry);
#endif

    if(entry == handle->rootEntry)
    {
        //the current rootEntry entry will be deleted, so the next entry becomes the new rootEntry
        //for a single entry this should not move the rootEntry, cause next is the rootEntry itself

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED    
        entry->nextEntry->previousEntry = entry->nextEntry;
#endif
        handle->rootEntry = entry->nextEntry;
    }
    else if(entry->nextEntry == entry)
    {
        //this is the last Entry in the buffer

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED    
        entry->previousEntry->nextEntry = entry->previousEntry;
#else
        previousEntry->nextEntry = previousEntry;
#endif
    }
#ifndef LINKED_LIST_BUFFER_DUALLY_LINKED    
    else if(previousEntry == NULL)
    {
        //Given Buffer not a part of active linked List
        LINKED_LIST_ASSERT(previousEntry != NULL);
        return false;
    }
#endif
    else
    {
        //Entry in the middle of the linked list
#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED    
        entry->previousEntry->nextEntry = entry->nextEntry;
        entry->nextEntry->previousEntry = entry->previousEntry;
#else
        previousEntry->nextEntry = entry->nextEntry;
#endif
    }

    //invalidate the handle
    entry->nextEntry = NULL;

#ifdef LINKED_LIST_BUFFER_DUALLY_LINKED    
    entry->previousEntry = NULL;
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

  if((linkedListArithmeticDataType_t)(ptr) < (linkedListArithmeticDataType_t)(&handle->entryPool[0]))
  {
      return NULL;
  }

  for(linkedListArithmeticDataType_t i = 1; i <= handle->entryPoolSize; i++)
  {
      if((linkedListArithmeticDataType_t)(ptr) < (linkedListArithmeticDataType_t)(&handle->entryPool[i]))
      {
          return &handle->entryPool[i-1];
      }
  }
  return NULL;
}
#endif // NOT(LINKED_LIST_ONLY_PROTOTYPE_DECLARATION)



