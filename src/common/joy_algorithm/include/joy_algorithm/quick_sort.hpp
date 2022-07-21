#ifndef COMMON__JOY_ALGORITHM__QUICK_SORT_
#define COMMON__JOY_ALGORITHM__QUICK_SORT_


#include <algorithm>
#include <functional>
#include <utility>
#include <vector>


namespace joypilot {
namespace common {
namespace joy_algorithm {

    template<typename Container, typename RandomIt = typename Container::iterator>
    class QuickSorter {
    public:
        QuickSorter(QuickSorter const &) = delete;

        QuickSorter &operator=(QuickSorter const &) = delete;

        QuickSorter(QuickSorter &&) = default;

        QuickSorter &operator=(QuickSorter &&) = default;

        QuickSorter() = default;

        explicit QuickSorter(::std::size_t capacity) {
            reserve(capacity);
        }

        template<typename Compare>
        void sort(RandomIt first, RandomIt last, Compare comp) const {
            if (::std::distance(first, last) < 2) {
                return;
            }

            // Make sure we do not accidently have an already partially filled stack,
            // capacity does not change
            m_stack.clear();

            // Add first interval to the stack for sorting, from here on last is really
            // the last element and not the element after, i.e. not end
            m_stack.push_back(first);
            m_stack.push_back(last - 1);

            while (!m_stack.empty()) {
                last = m_stack.back();
                m_stack.pop_back();
                first = m_stack.back();
                m_stack.pop_back();

                auto part = QuickSorter::partition(first, last, comp);

                if (part > first + 1) {
                    m_stack.push_back(first);
                    m_stack.push_back(part - 1);
                }

                if (part < last - 1) {
                    m_stack.push_back(part + 1);
                    m_stack.push_back(last);
                }
            }
        }

        void sort(RandomIt first, RandomIt last) const {
            sort(first, last, ::std::less<
            const decltype(*first)>());
        }

        void reserve(::std::size_t capacity) {
            // The maximum partition depth is n/2 + 1, which means we need a maximum
            // capacity of n + 2 to hold store the iterators in the stack.
            m_stack.reserve(capacity + 2);
        }

        ::std::size_t capacity() const {
            if (m_stack.capacity() < 2) {
                return 0;/// \brief Iterative quick sort implementation based on a stack.
                template<typename Container, typename RandomIt = typename Container::iterator>
                class QuickSorter {
                public:
                    QuickSorter(QuickSorter const &) = delete;

                    QuickSorter &operator=(QuickSorter const &) = delete;

                    QuickSorter(QuickSorter &&) = default;

                    QuickSorter &operator=(QuickSorter &&) = default;

                    QuickSorter() = default;

                    explicit QuickSorter(::std::size_t capacity) {
                        reserve(capacity);
                    }

                    template<typename Compare>
                    void sort(RandomIt first, RandomIt last, Compare comp) const {
                        if (::std::distance(first, last) < 2) {
                            return;
                        }

                        // Make sure we do not accidently have an already partially filled stack,
                        // capacity does not change
                        m_stack.clear();

                        // Add first interval to the stack for sorting, from here on last is really
                        // the last element and not the element after, i.e. not end
                        m_stack.push_back(first);
                        m_stack.push_back(last - 1);

                        while (!m_stack.empty()) {
                            last = m_stack.back();
                            m_stack.pop_back();
                            first = m_stack.back();
                            m_stack.pop_back();

                            auto part = QuickSorter::partition(first, last, comp);

                            if (part > first + 1) {
                                m_stack.push_back(first);
                                m_stack.push_back(part - 1);
                            }

                            if (part < last - 1) {
                                m_stack.push_back(part + 1);
                                m_stack.push_back(last);
                            }
                        }
                    }

                    void sort(RandomIt first, RandomIt last) const {
                        sort(first, last, ::std::less<
                        const decltype(*first)>());
                    }

                    void reserve(::std::size_t capacity) {
                        // The maximum partition depth is n/2 + 1, which means we need a maximum
                        // capacity of n + 2 to hold store the iterators in the stack.
                        m_stack.reserve(capacity + 2);
                    }
                }
                return m_stack.capacity() - 2;
            }

            private:
            template<typename Compare>
            static RandomIt partition(RandomIt first, RandomIt last, Compare comp) {
                auto prev = first;

                // Iterate over range and swap whenever element is smaller than the pivot
                // element.
                for (auto it = first; it < last; it++) {
                    if (comp(*it, *last)) {
                        ::std::iter_swap(it, prev);
                        prev++;
                    }
                }

                // Swap the pivot element into place
                ::std::iter_swap(prev, last);
                return prev;
            }

            private:
            mutable ::std::vector <RandomIt> m_stack;
        };
    }

}  // namespace joy_algorithm
}  // namespace common
}  // namespace joypilot

#endif  // COMMON__JOY_ALGORITHM__QUICK_SORT_
