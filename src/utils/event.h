#ifndef EVENT_H
#define EVENT_H

#include <set>
#include <list>



template<class F, typename... T>
class Event
{
    friend F;



/*    T y p e s    */

public:

    typedef void(* EventHandler)(void*, T...);


private:

    typedef std::pair<void*, EventHandler> BoundEventHandler;



/*    V a r i a b l e s    */

private:

    std::set<BoundEventHandler> boundEventHandlerSet;



/*    M e t h o d s    */

public:

    void registerEventHandler(void* listener, EventHandler eventHandler)
    {
        BoundEventHandler boundEventHandler(listener, eventHandler);
        this->boundEventHandlerSet.insert(boundEventHandler);
    }


    template<class C>
    void registerEventHandler(C* listener, void(C::* eventHandler)(T...))
    {
        BoundEventHandler boundEventHandler((void*) listener, (EventHandler) eventHandler);
        this->boundEventHandlerSet.insert(boundEventHandler);
    }


    void unregisterEventHandler(void* listener, EventHandler eventHandler)
    {
        BoundEventHandler boundEventHandler(listener, eventHandler);
        this->boundEventHandlerSet.erase(boundEventHandler);
    }


    template<class C>
    void unregisterEventHandler(C* listener, void(C::* eventHandler)(T...))
    {
        BoundEventHandler boundEventHandler((void*) listener, (EventHandler) eventHandler);
        this->boundEventHandlerSet.erase(boundEventHandler);
    }


private:

    void removeAllRegisteredEventHandlers()
    {
        this->boundEventHandlerSet.clear();
    }


    void triggerEvent(T...args)
    {
        for (BoundEventHandler boundEventHandler : this->boundEventHandlerSet)
        {
            void*        listener     = boundEventHandler.first;
            EventHandler eventHandler = boundEventHandler.second;
            eventHandler(listener, args...);
        }
    }
};




template<class F, typename R, typename... T>
class EventWithReturnValue
{
    friend F;



/*    T y p e s    */

public:

    typedef R(* EventHandler)(void*, T...);


private:

    typedef std::pair<void*, EventHandler> BoundEventHandler;



/*    V a r i a b l e s    */

private:

    std::set<BoundEventHandler> boundEventHandlerSet;



/*    M e t h o d s    */

public:

    void registerEventHandler(void* listener, EventHandler eventHandler)
    {
        BoundEventHandler boundEventHandler(listener, eventHandler);
        this->boundEventHandlerSet.insert(boundEventHandler);
    }


    template<class C>
    void registerEventHandler(C* listener, R(C::* eventHandler)(T...))
    {
        BoundEventHandler boundEventHandler((void*) listener, (EventHandler) eventHandler);
        this->boundEventHandlerSet.insert(boundEventHandler);
    }


    void unregisterEventHandler(void* listener, EventHandler eventHandler)
    {
        BoundEventHandler boundEventHandler(listener, eventHandler);
        this->boundEventHandlerSet.erase(boundEventHandler);
    }


    template<class C>
    void unregisterEventHandler(C* listener, R(C::* eventHandler)(T...))
    {
        BoundEventHandler boundEventHandler((void*) listener, (EventHandler) eventHandler);
        this->boundEventHandlerSet.erase(boundEventHandler);
    }


private:

    void removeAllRegisteredEventHandlers()
    {
        this->boundEventHandlerSet.clear();
    }


    std::list<R> triggerEvent(T...args)
    {
        std::list<R> returnValueList;
        for (BoundEventHandler boundEventHandler : this->boundEventHandlerSet)
        {
            void*        listener     = boundEventHandler.first;
            EventHandler eventHandler = boundEventHandler.second;
            returnValueList.push_back(eventHandler(listener, args...));
        }
        return returnValueList;
    }
};

#endif // EVENT_H
