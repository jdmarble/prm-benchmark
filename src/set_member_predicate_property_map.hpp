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

}
