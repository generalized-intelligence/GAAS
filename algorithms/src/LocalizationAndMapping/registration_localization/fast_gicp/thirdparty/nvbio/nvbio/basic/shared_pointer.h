/*
 * nvbio
 * Copyright (c) 2011-2014, NVIDIA CORPORATION. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *    * Redistributions of source code must retain the above copyright
 *      notice, this list of conditions and the following disclaimer.
 *    * Redistributions in binary form must reproduce the above copyright
 *      notice, this list of conditions and the following disclaimer in the
 *      documentation and/or other materials provided with the distribution.
 *    * Neither the name of the NVIDIA CORPORATION nor the
 *      names of its contributors may be used to endorse or promote products
 *      derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NVIDIA CORPORATION BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <typeinfo>
#include <iosfwd>    // for basic_ostream<>
#include <algorithm> // for std::swap

namespace nvbio {

#ifdef WIN32
#else
//#define SharedPointer std::tr1::shared_ptr
#pragma GCC poison shared_ptr
#endif

    // A shared pointer, with an interface same as the boost/tr1 one
    // but templatized on the counter's type as well. This is so you
    // can have it thread safe or non-thread-safe.

    typedef long CounterT_default;

    namespace internals {
        template <typename T>
        struct Deleter {
            typedef void result_type;
            typedef T*   argument_type;

            void operator() (argument_type p) const { delete p; }
        };

        template<typename CounterT>
        class CountedBase {
            public:
                typedef CounterT Counter_Type;
            CountedBase() :
                m_shcount(1),
                m_wkcount(1)
            {}

            virtual ~CountedBase() {}

            /// called when m_shcount goes to 0
            virtual void dispose() = 0;

            /// called when m_wkcount goes to 0
            virtual void destroy() { delete this; }

            virtual void* get_deleter(const std::type_info&) = 0;

            void add_ref_copy() { ++m_shcount; }

            void add_ref_lock() { ++m_shcount; }

            void release() {
                if (!--m_shcount) {
                    dispose();
                    weak_release();
                }
            }

            void weak_add_ref() { ++m_wkcount; }
            void weak_release() {
                if (!--m_wkcount) {
                    destroy();
                }
            }

            Counter_Type use_count() const { return m_shcount; }

            private:
                // No copies nor assignments
                CountedBase(const CountedBase&);
                CountedBase& operator=(const CountedBase&);

                Counter_Type m_shcount; //< shared pointers count
                Counter_Type m_wkcount; //< weak pointers count + (m_shcount + 1)
        };

        template<typename PtrT, typename DeleterT, typename CounterT>
        class CountedBase_impl : public CountedBase<CounterT>
        {
            public:
                CountedBase_impl(PtrT p, DeleterT d) :
                    CountedBase<CounterT>(),
                    m_ptr(p),
                    m_del(d)
                {}

                virtual void dispose() { m_del(m_ptr); }

                virtual void* get_deleter(const std::type_info& ti) {
                    return (ti == typeid(DeleterT)) ? &m_del : NULL;
                }

            private:
                // No copies nor assignments
                CountedBase_impl(const CountedBase<CounterT>&);
                CountedBase_impl& operator=(const CountedBase<CounterT>&);

                PtrT     m_ptr;
                DeleterT m_del;
        };

        template<typename CounterT> class WeakCount;

        template<typename CounterT>
        class SharedCount {
            public:
                typedef typename CountedBase<CounterT>::Counter_Type Counter_Type;

                SharedCount() :
                    m_pi(NULL)
                {}

                template<typename PtrT, typename DeleterT>
                SharedCount(PtrT p, DeleterT d) :
                    m_pi(NULL)
                {
                    try {
                        m_pi = new CountedBase_impl<PtrT, DeleterT, CounterT>(p, d);
                    } catch (...) {
                        d(p);
                        throw;
                    }
                }

                // swallowed support for auto_ptr<T>, should go here

                explicit SharedCount(const WeakCount<CounterT>& wc);

                ~SharedCount() {
                    if (m_pi)
                        m_pi->release();
                }

                SharedCount(const SharedCount& sc) :
                    m_pi(sc.m_pi)
                {
                    if (m_pi)
                        m_pi->add_ref_copy();
                }

                SharedCount& operator= (const SharedCount& sc)
                {
                    CountedBase<CounterT>* _tmp = sc.m_pi;

                    if (_tmp != m_pi) {
                        if (_tmp)
                            _tmp->add_ref_copy();
                        if (m_pi)
                            m_pi->release();
                        m_pi = _tmp;
                    }
                    return *this;
                }

                void swap(SharedCount& sc)
                {
                    CountedBase<CounterT>* _tmp = sc.m_pi;
                    sc.m_pi = m_pi;
                    m_pi = _tmp;
                }

                Counter_Type use_count() const
                {
                    return m_pi ? m_pi->use_count() : CountedBase<CounterT>::Counter_Type(0);
                }

                bool unique() const { return use_count() == 1;}

                friend inline bool operator== (const SharedCount& a, const SharedCount& b)
                {
                    return a.m_pi == b.m_pi;
                }

                friend inline bool operator< (const SharedCount& a, const SharedCount& b)
                {
                    return a.m_pi < b.m_pi;
                }

                void* get_deleter(const std::type_info& ti) const {
                    return m_pi ? m_pi->get_deleter(ti) : NULL;
                }
            private:
                friend class WeakCount<CounterT>;
                CountedBase<CounterT>* m_pi;
        };

        template<typename CounterT>
        class WeakCount {
            public:
                typedef typename CountedBase<CounterT>::Counter_Type Counter_Type;

                WeakCount() :
                    m_pi(NULL)
                {}

                WeakCount(const SharedCount<CounterT>& sc) :
                    m_pi(sc.m_pi)
                {
                    if (m_pi)
                        m_pi->weak_add_ref();
                }

                WeakCount(const WeakCount& wc) :
                    m_pi(wc.m_pi)
                {
                    if (m_pi)
                        m_pi->weak_add_ref();
                }

                ~WeakCount() {
                    if (m_pi)
                        m_pi->weak_release();
                }

                WeakCount& operator= (const SharedCount<CounterT>& sc)
                {
                    CountedBase<CounterT>* _tmp = sc.m_pi;

                    if (_tmp)
                        _tmp->weak_add_ref();
                    if (m_pi)
                        m_pi->weak_release();
                    m_pi = _tmp;

                    return *this;
                }

                WeakCount& operator= (const WeakCount& wc)
                {
                    CountedBase<CounterT>* _tmp = wc.m_pi;

                    if (_tmp)
                        _tmp->weak_add_ref();
                    if (m_pi)
                        m_pi->weak_release();
                    m_pi = _tmp;

                    return *this;
                }

                void swap(WeakCount& wc)
                {
                    CountedBase<CounterT>* _tmp = wc.m_pi;
                    wc.m_pi = m_pi;
                    m_pi = _tmp;
                }

                Counter_Type use_count() const
                {
                    return m_pi ? m_pi->use_count() : Counter_Type(0);
                }

                friend inline bool operator== (const WeakCount& a, const WeakCount& b)
                {
                    return a.m_pi == b.m_pi;
                }

                friend inline bool operator< (const WeakCount& a, const WeakCount& b)
                {
                    return a.m_pi < b.m_pi;
                }

                void* get_deleter(const std::type_info& ti) const {
                    return m_pi ? m_pi->get_deleter(ti) : NULL;
                }
            private:
                friend class SharedCount<CounterT>;
                CountedBase<CounterT>* m_pi;
        };

        template<typename CounterT>
        inline
        SharedCount<CounterT>::SharedCount(const WeakCount<CounterT>& wc) :
            m_pi(wc.m_pi)
        {
            if (m_pi)
                m_pi->add_ref_lock();
            else {
                // FIXME: throw bad weak pointer
            }
        }

        template<class T> struct SharedPointerTraits
        {
            typedef T& reference;
        };

        template<> struct SharedPointerTraits<void>
        {
            typedef void reference;
        };

        template<> struct SharedPointerTraits<void const>
        {
            typedef void reference;
        };

        template<> struct SharedPointerTraits<void volatile>
        {
            typedef void reference;
        };

        template<> struct SharedPointerTraits<void const volatile>
        {
            typedef void reference;
        };

        struct static_cast_marker {};
        struct const_cast_marker {};
        struct dynamic_cast_marker {};
        struct polymorphic_cast_marker {};
    }

    template<typename T, typename CounterT> class WeakPointer;

    template<typename T, typename CounterT = CounterT_default>
    class SharedPointer
    {
            typedef typename internals::SharedPointerTraits<T>::reference _Reference;
        public:
            typedef T                                                       element_type;
            typedef typename internals::SharedCount<CounterT>::Counter_Type Counter_Type;

            SharedPointer() :
                m_ptr(NULL),
                m_count()
            {}

            template<typename U>
            explicit SharedPointer(U* p) :
                m_ptr(p),
                m_count(p, internals::Deleter<T>())
            {}

            template<typename U, typename DeleterT>
            explicit SharedPointer(U* p, DeleterT d) :
                m_ptr(p),
                m_count(p, d)
            {}

            // generated copy constructor, assignment, destructor are fine

            template<typename U>
            SharedPointer(const SharedPointer<U>& other) :
                m_ptr(other.m_ptr),
                m_count(other.m_count)
            {

            }

            template<typename U>
            SharedPointer(const WeakPointer<U, CounterT>& other) :
                m_count(other.m_count)
            {
                // the constructor of m_count may throw if the reference counter
                // in the weak pointer is 0, which means the pointer is gone.
                // In that case we never get here.
                m_ptr = other.m_ptr;
            }

            // swallowed support for auto_ptr<T>, should go here

            template<typename U>
            SharedPointer(const SharedPointer<U>& other, internals::static_cast_marker) :
                m_ptr(static_cast<element_type*>(other.m_ptr)),
                m_count(other.m_count)
            {}

            template<typename U>
            SharedPointer(const SharedPointer<U>& other, internals::const_cast_marker) :
                m_ptr(const_cast<element_type*>(other.m_ptr)),
                m_count(other.m_count)
            {}

            template<typename U>
            SharedPointer(const SharedPointer<U>& other, internals::dynamic_cast_marker) :
                m_ptr(dynamic_cast<element_type*>(other.m_ptr)),
                m_count(other.m_count)
            {
                // the cast may fail at runtime
                if (m_ptr == NULL)
                    m_count = internals::SharedCount<CounterT>();
            }

            template<typename U>
            SharedPointer& operator= (const SharedPointer<U>& other)
            {
                m_ptr   = other.m_ptr;
                m_count = other.m_count;
                return *this;
            }

            template<typename U>
            SharedPointer& operator= (U* p)
            {
                reset( p );
                return *this;
            }

            // swallowed support for auto_ptr<T>, should go here

            void reset()
            {
                SharedPointer().swap(*this);
            }

            template<typename U>
            void reset(U* p)
            {
                if (p != m_ptr)
                    SharedPointer(p).swap(*this);
            }

            template<typename U, typename UDeleterT>
            void reset(U* p, UDeleterT d)
            {
                if (p != m_ptr)
                    SharedPointer(p, d).swap(*this);
            }

            _Reference    operator*()  const { return *m_ptr; }
            element_type* operator->() const { return m_ptr; }
            element_type* get()        const { return m_ptr; }

        private:
            // conversion to "bool"
            typedef element_type* SharedPointer::*BooleanType;
        public:
            operator BooleanType() const
            {
                return m_ptr ? &SharedPointer::m_ptr : NULL;
            }

            bool unique() const { return m_count.unique(); }

            Counter_Type use_count() const { return m_count.use_count(); }

            void swap(SharedPointer<element_type>& other)
            {
                std::swap(m_ptr, other.m_ptr);
                m_count.swap(other.m_count);
            }

            // for private usage. Use free standing get_deleter() instead
            void* _M_get_deleter(const std::type_info& ti) const
            {
                return m_count.get_deleter(ti);
            }

        private:
            template<typename U>
            bool _M_less(const SharedPointer<U>& rhs) const
            {
                return m_count < rhs.m_count;
            }

            // All SharedPointer's are friends
            template<typename U, typename UCounterT> friend class SharedPointer;
            // with all WeakPointer's too
            template<typename U, typename UCounterT> friend class WeakPointer;

            template<typename U>
            friend inline bool operator==(const SharedPointer& a, const SharedPointer<U>& b)
            {
                return a.get() == b.get();
            }

            template<typename U>
            friend inline bool operator!=(const SharedPointer& a, const SharedPointer<U>& b)
            {
                return a.get() != b.get();
            }

            template<typename U>
            friend inline bool operator<(const SharedPointer& a, const SharedPointer<U>& b)
            {
                return a._M_less(b);
            }

            element_type*         m_ptr;
            internals::SharedCount<CounterT> m_count;
    };


    template<typename T, typename CounterT>
    inline void swap(SharedPointer<T, CounterT>& a, SharedPointer<T, CounterT>& b)
    {
        a.swap(b);
    }

    template<typename T, typename U, typename CounterT>
    SharedPointer<T, CounterT> static_pointer_cast(const SharedPointer<U, CounterT>& r)
    {
        return SharedPointer<T, CounterT>(r, internals::static_cast_marker());
    }

    template<typename T, typename U, typename CounterT>
    SharedPointer<T, CounterT> const_pointer_cast(const SharedPointer<U, CounterT>& r)
    {
        return SharedPointer<T, CounterT>(r, internals::const_cast_marker());
    }

    template<typename T, typename U, typename CounterT>
    SharedPointer<T, CounterT> dynamic_pointer_cast(const SharedPointer<U, CounterT>& r)
    {
        return SharedPointer<T, CounterT>(r, internals::dynamic_cast_marker());
    }

    template<typename CharT, typename TraitsT, typename T, typename CounterT>
    std::basic_ostream<CharT, TraitsT>&
    operator<<(std::basic_ostream<CharT, TraitsT>& os, const SharedPointer<T, CounterT>& p)
    {
        return os << p.get();
    }

    template<typename DeleterT, typename T, typename CounterT>
    inline DeleterT* get_deleter(const SharedPointer<T, CounterT>& p)
    {
        return static_cast<DeleterT*>(p._M_get_deleter(typeid(DeleterT)));
    }

    template<typename T, typename CounterT = long>
    class WeakPointer
    {
        public:
            typedef T                                                        element_type;
            typedef typename internals::SharedCount<CounterT>::Counter_Type  Counter_Type;

            WeakPointer() :
                m_ptr(NULL),
                m_count()
            {}

            // generated copy constructor, assignment, destructor are fine

            // this implementation is required because the pointer could be
            // invalidated during construction, so we lock, copy, unlock
            template<typename U>
            WeakPointer(const WeakPointer<U>& other) :
                m_count(other.m_count)
            {
                m_ptr = other.lock().get();
            }

            template<typename U>
            WeakPointer(const SharedPointer<U, CounterT>& other) :
                m_ptr(other.m_ptr),
                m_count(other.m_count)
            {
            }

            template<typename U>
            WeakPointer& operator=(const WeakPointer<U>& other)
            {
                m_ptr   = other.lock().get();
                m_count = other.m_count;
                return *this;
            }

            template<typename U>
            WeakPointer& operator=(const SharedPointer<U>& other)
            {
                m_ptr   = other.m_ptr;
                m_count = other.m_count;
                return *this;
            }

            SharedPointer<element_type> lock() const {
                // there is a possible race-condition here. See notes in the
                // BOOST implementation for a solution
                return expired() ? SharedPointer<element_type>() : SharedPointer<element_type>(*this);
            }

            Counter_Type use_count() const { return m_count.use_count(); }

            bool expired() const { return m_count.use_count() == 0; }

            void reset() { WeakPointer().swap(*this); }

            void swap(WeakPointer& other)
            {
                std::swap(m_ptr, other.m_ptr);
                m_count.swap(other.m_count);
            }

        private:
            template<typename U>
            bool _M_less(const SharedPointer<U>& rhs) const
            {
                return m_count < rhs.m_count;
            }

            // All SharedPointer's are friends
            template<typename U, typename UCounterT> friend class SharedPointer;
            // with all WeakPointer's too
            template<typename U, typename UCounterT> friend class WeakPointer;

            template<typename U>
            friend inline bool operator<(const WeakPointer& a, const WeakPointer<U>& b)
            {
                return a._M_less(b);
            }

            element_type*                   m_ptr;
            internals::WeakCount<CounterT>  m_count;
    };

    template<typename T, typename CounterT>
    inline void swap(WeakPointer<T, CounterT>& a, WeakPointer<T, CounterT>& b)
    {
        a.swap(b);
    }

    // swallowed enable_shared_from_this

} // namespace nvbio

namespace std {
    template<typename T, typename CounterT>
    inline void swap(::nvbio::SharedPointer<T, CounterT>& a, ::nvbio::SharedPointer<T, CounterT>& b)
    {
        a.swap(b);
    }

    template<typename T, typename CounterT>
    inline void swap(::nvbio::WeakPointer<T, CounterT>& a, ::nvbio::WeakPointer<T, CounterT>& b)
    {
        a.swap(b);
    }
}
