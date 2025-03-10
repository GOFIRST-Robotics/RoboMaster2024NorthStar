// /*****************************************************************************/
// /********** !!! WARNING: CODE GENERATED BY TAPROOT. DO NOT EDIT !!! **********/
// /*****************************************************************************/

// /*
//  * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
//  *
//  * This file is part of Taproot.
//  *
//  * Taproot is free software: you can redistribute it and/or modify
//  * it under the terms of the GNU General Public License as published by
//  * the Free Software Foundation, either version 3 of the License, or
//  * (at your option) any later version.
//  *
//  * Taproot is distributed in the hope that it will be useful,
//  * but WITHOUT ANY WARRANTY; without even the implied warranty of
//  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  * GNU General Public License for more details.
//  *
//  * You should have received a copy of the GNU General Public License
//  * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
//  */

// #include "additionalTapResources/wrapped_float.hpp"

// namespace tap
// {
// namespace algorithms
// {
// WrappedFloat::WrappedFloat(const float value, const float lowerBound, const float upperBound)
//     : wrapped(value),
//       lowerBound(lowerBound),
//       upperBound(upperBound)
// {
//     assert(upperBound > lowerBound);

//     wrapValue();
// }

// void WrappedFloat::operator+=(const WrappedFloat& other)
// {
//     assertBoundsEqual(other);

//     this->wrapped += other.wrapped;
//     if (this->wrapped > this->upperBound) this->wrapped -= (this->upperBound - this->lowerBound);
//     this->revolutions += other.revolutions;
// }

// void WrappedFloat::operator-=(const WrappedFloat& other)
// {
//     assertBoundsEqual(other);

//     this->wrapped -= other.wrapped;
//     if (this->wrapped < this->lowerBound) this->wrapped += (this->upperBound - this->lowerBound);
//     this->revolutions -= other.revolutions;
// }

// WrappedFloat WrappedFloat::operator+(const WrappedFloat& other) const
// {
//     assertBoundsEqual(other);

//     WrappedFloat temp(*this);
//     temp += other;
//     return temp;
// }

// WrappedFloat WrappedFloat::operator-(const WrappedFloat& other) const
// {
//     assertBoundsEqual(other);

//     WrappedFloat temp = *this;
//     temp -= other;
//     return temp;
// }

// void WrappedFloat::operator+=(float value)
// {
//     *this += WrappedFloat(value, this->lowerBound, this->upperBound);
// }

// void WrappedFloat::operator-=(float value)
// {
//     *this -= WrappedFloat(value, this->lowerBound, this->upperBound);
// }

// WrappedFloat WrappedFloat::operator+(float value) const
// {
//     return *this + WrappedFloat(value, this->lowerBound, this->upperBound);
// }

// WrappedFloat WrappedFloat::operator-(float value) const
// {
//     return *this - WrappedFloat(value, this->lowerBound, this->upperBound);
// }

// float WrappedFloat::minDifference(const WrappedFloat& other) const
// {
//     assertBoundsEqual(other);

//     float interval = this->getUpperBound() - this->getLowerBound();
//     float difference_between = other.getWrappedValue() - this->getWrappedValue();
//     float difference_around =
//         difference_between + ((difference_between < 0) ? interval : -interval);
//     return (abs(difference_between) < abs(difference_around)) ? difference_between
//                                                               : difference_around;
// }

// float WrappedFloat::minDifference(const float& unwrappedValue) const
// {
//     return minDifference(WrappedFloat(unwrappedValue, this->lowerBound, this->upperBound));
// }

// WrappedFloat WrappedFloat::minInterpolate(const WrappedFloat& other, const float alpha) const
// {
//     assertBoundsEqual(other);

//     float halfInterval = (this->upperBound - other.lowerBound) / 2;
//     float rawInterpolation = this->wrapped * alpha + other.wrapped * (1 - alpha);

//     if (abs(this->wrapped - other.wrapped) <= halfInterval)
//     {
//         return WrappedFloat(rawInterpolation, this->lowerBound, this->upperBound);
//     }
//     else
//     {
//         if (rawInterpolation > this->lowerBound + halfInterval)
//             return WrappedFloat(
//                 rawInterpolation - halfInterval,
//                 this->lowerBound,
//                 this->upperBound);
//         else
//             return WrappedFloat(
//                 rawInterpolation + halfInterval,
//                 this->lowerBound,
//                 this->upperBound);
//     }
// }

// void WrappedFloat::shiftBounds(const float shiftMagnitude)
// {
//     upperBound += shiftMagnitude;
//     lowerBound += shiftMagnitude;
//     wrapValue();
// }

// void WrappedFloat::wrapValue()
// {
//     float oldValue = wrapped;
//     if (oldValue < lowerBound)
//     {
//         this->wrapped = upperBound + fmodf(oldValue - upperBound, upperBound - lowerBound);
//     }
//     else if (oldValue >= upperBound - 1E-5)
//     {
//         this->wrapped = lowerBound + fmodf(oldValue - lowerBound, upperBound - lowerBound);
//     }
//     this->revolutions += static_cast<int>((oldValue - lowerBound) / (upperBound - lowerBound));
// }

// float WrappedFloat::limitValue(
//     const WrappedFloat& valueToLimit,
//     const float min,
//     const float max,
//     int* status)
// {
//     WrappedFloat minWrapped(min, valueToLimit.lowerBound, valueToLimit.upperBound);
//     WrappedFloat maxWrapped(max, valueToLimit.lowerBound, valueToLimit.upperBound);
//     return limitValue(valueToLimit, minWrapped, maxWrapped, status);
// }

// float WrappedFloat::limitValue(
//     const WrappedFloat& valueToLimit,
//     const WrappedFloat& min,
//     const WrappedFloat& max,
//     int* status)
// {
//     WrappedFloat::assertBoundsEqual(min, max);
//     WrappedFloat::assertBoundsEqual(valueToLimit, min);

//     if (min.getWrappedValue() == max.getWrappedValue())
//     {
//         return valueToLimit.getWrappedValue();
//     }
//     if ((min.getWrappedValue() < max.getWrappedValue() &&
//          (valueToLimit.getWrappedValue() > max.getWrappedValue() ||
//           valueToLimit.getWrappedValue() < min.getWrappedValue())) ||
//         (min.getWrappedValue() > max.getWrappedValue() &&
//          valueToLimit.getWrappedValue() > max.getWrappedValue() &&
//          valueToLimit.getWrappedValue() < min.getWrappedValue()))
//     {
//         // valueToLimit is not "within" min and max
//         float targetMinDifference = valueToLimit.minDifference(min);
//         float targetMaxDifference = valueToLimit.minDifference(max);

//         if (fabs(targetMinDifference) < fabs(targetMaxDifference))
//         {
//             *status = 1;
//             return min.getWrappedValue();
//         }
//         else
//         {
//             *status = 2;
//             return max.getWrappedValue();
//         }
//     }
//     else
//     {
//         *status = 0;
//         return valueToLimit.getWrappedValue();
//     }
// }

// }  // namespace algorithms

// }  //  namespace tap
