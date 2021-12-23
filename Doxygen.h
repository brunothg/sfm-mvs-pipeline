/**
 * Niemals importieren. Stellt lediglich Definitionen für Doxygen zur Verfügung, die ansonsten fehlen würden.
 */

namespace std {
    template<class T> class shared_ptr { 
        public: 
            T *ptr;
    };

    template<class T> class unique_ptr { 
        public: 
            T *ptr;
    };
}