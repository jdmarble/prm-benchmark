#include <boost/property_map/property_map.hpp>

namespace boost {

    template <typename SetContainer>
    class const_set_member_predicate_property_map
        : public boost::put_get_helper<
            bool,
            const_set_member_predicate_property_map<SetContainer> >
    {
    private:

        typedef SetContainer C;
        C const* m_c;

    public:

        typedef bool reference;
        typedef readable_property_map_tag category;
        const_set_member_predicate_property_map() : m_c(NULL) { }
        const_set_member_predicate_property_map(const C& c) : m_c(&c) { }
        reference operator[](const typename SetContainer::value_type& k) const {
            return m_c->find(k) != m_c->end();
        }

    };

    namespace detail {
        // Proxy class.
        template <typename SetContainer>
        class reference_wrapper
        {
        private:
            typedef typename SetContainer::value_type value_type;
            SetContainer* m_c;
            const value_type m_key;

        public:

            reference_wrapper(SetContainer& c, const value_type& key)
                : m_c(&c), m_key(key) {}

            bool operator=(const bool add)
            {
                if (add) m_c->insert(m_key);
                return add;
            }

            operator bool() const {
                return m_c->find(m_key) != m_c->end();
            }
        };
    }// namespace detail

    template <typename SetContainer>
    class set_member_predicate_property_map
        : public boost::put_get_helper<
            bool,
            set_member_predicate_property_map<SetContainer> >
    {
    private:

        SetContainer* m_c;        

    public:

        typedef detail::reference_wrapper<SetContainer> reference;
        typedef bool value_type;
        typedef read_write_property_map_tag category;
        set_member_predicate_property_map() : m_c(NULL) { }
        set_member_predicate_property_map(SetContainer& c) : m_c(&c) { }

        bool operator[](const typename SetContainer::value_type& k) const {
            return m_c->find(k) != m_c->end();
        }

        reference operator[](const typename SetContainer::value_type& k) {
            return reference(*m_c, k);
        }

    };

}
