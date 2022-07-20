#include <gtest/gtest.h>
#include <vector>
#include "avp_algorithm/avp_algorithm.hpp"

template<typename Container, typename RandomIt = typename Container::iterator>
using QuickSorter = ::autoware::common::algorithm::QuickSorter<Container, RandomIt>;

TEST(quick_sort_iterative, empty) {
::std::vector<int32_t> vector;
QuickSorter<::std::vector<int32_t>> sorter;
sorter.sort(vector.begin(), vector.end());
ASSERT_EQ(vector, ::std::vector<int32_t>({}));
ASSERT_EQ(sorter.capacity(), 0UL);
}

TEST(quick_sort_iterative, single_elem) {
::std::vector<int32_t> vector = {42};
QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
ASSERT_EQ(sorter.capacity(), 1UL);
sorter.sort(vector.begin(), vector.end());
ASSERT_EQ(vector, ::std::vector<int32_t>({42}));
ASSERT_EQ(sorter.capacity(), 1UL);
}

TEST(quick_sort_iterative, two_elems) {
::std::vector<int32_t> vector = {42, 43};
QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
ASSERT_EQ(sorter.capacity(), 2UL);
sorter.sort(vector.begin(), vector.end());
ASSERT_EQ(vector, ::std::vector<int32_t>({42, 43}));
ASSERT_EQ(sorter.capacity(), 2UL);
}

TEST(quick_sort_iterative, already_sorted) {
::std::vector<int32_t> vector = {1, 2, 3, 4, 5, 6};
QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
ASSERT_EQ(sorter.capacity(), 6UL);
sorter.sort(vector.begin(), vector.end());
ASSERT_EQ(vector, ::std::vector<int32_t>({1, 2, 3, 4, 5, 6}));
ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(quick_sort_iterative, descending) {
::std::vector<int32_t> vector = {6, 5, 4, 3, 2, 1};
QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
ASSERT_EQ(sorter.capacity(), 6UL);
sorter.sort(vector.begin(), vector.end());
ASSERT_EQ(vector, ::std::vector<int32_t>({1, 2, 3, 4, 5, 6}));
ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(quick_sort_iterative, random) {
::std::vector<int32_t> vector = {3, 5, 1, 6, 4, 2};
QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
ASSERT_EQ(sorter.capacity(), 6UL);
sorter.sort(vector.begin(), vector.end());
ASSERT_EQ(vector, ::std::vector<int32_t>({1, 2, 3, 4, 5, 6}));
ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(quick_sort_iterative, sub_range_begin) {
::std::vector<int32_t> vector = {3, 5, 1, 6, 4, 2};
QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
ASSERT_EQ(sorter.capacity(), 6UL);
sorter.sort(vector.begin(), vector.end() - 2);
ASSERT_EQ(vector, ::std::vector<int32_t>({1, 3, 5, 6, 4, 2}));
ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(quick_sort_iterative, sub_range_end) {
::std::vector<int32_t> vector = {3, 5, 1, 6, 4, 2};
QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
ASSERT_EQ(sorter.capacity(), 6UL);
sorter.sort(vector.begin() + 2, vector.end());
ASSERT_EQ(vector, ::std::vector<int32_t>({3, 5, 1, 2, 4, 6}));
ASSERT_EQ(sorter.capacity(), 6UL);
}

TEST(quick_sort_iterative, sub_range) {
::std::vector<int32_t> vector = {3, 5, 1, 6, 4, 2};
QuickSorter<::std::vector<int32_t>> sorter(vector.capacity());
ASSERT_EQ(sorter.capacity(), 6UL);
sorter.sort(vector.begin() + 1, vector.end() - 3);
ASSERT_EQ(vector, ::std::vector<int32_t>({3, 1, 5, 6, 4, 2}));
ASSERT_EQ(sorter.capacity(), 6UL);
}