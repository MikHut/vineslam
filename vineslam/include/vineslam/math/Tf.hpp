#pragma once

#include <iostream>
#include <iomanip>
#include <array>

namespace vineslam
{
struct Tf
{
  Tf()
  {
    R_array_ = { 1., 0., 0., 0., 1., 0., 0., 0., 1. };
    t_array_ = { 0., 0., 0. };
  }

  Tf(const std::array<float, 9>& m_R, const std::array<float, 3>& m_t)
  {
    R_array_ = m_R;
    t_array_ = m_t;
  }

  Tf inverse()
  {
    std::array<float, 9> R_inverse = transposeRotation();
    std::array<float, 3> t_inverse{};
    t_inverse[0] = R_inverse[0] * (-t_array_[0]) + R_inverse[1] * (-t_array_[1]) + R_inverse[2] * (-t_array_[2]);
    t_inverse[1] = R_inverse[3] * (-t_array_[0]) + R_inverse[4] * (-t_array_[1]) + R_inverse[5] * (-t_array_[2]);
    t_inverse[2] = R_inverse[6] * (-t_array_[0]) + R_inverse[7] * (-t_array_[1]) + R_inverse[8] * (-t_array_[2]);

    Tf inv_tf(R_inverse, t_inverse);

    return inv_tf;
  }

  std::array<float, 9> transposeRotation()
  {
    std::array<float, 9> R_transpose = { R_array_[0], R_array_[3], R_array_[6], R_array_[1], R_array_[4],
                                         R_array_[7], R_array_[2], R_array_[5], R_array_[8] };

    return R_transpose;
  }

  Tf operator*(const Tf& m_tf)
  {
    std::array<float, 9> m_R{};
    std::array<float, 3> m_t{};

    m_R[0] = R_array_[0] * m_tf.R_array_[0] + R_array_[1] * m_tf.R_array_[3] + R_array_[2] * m_tf.R_array_[6];
    m_R[1] = R_array_[0] * m_tf.R_array_[1] + R_array_[1] * m_tf.R_array_[4] + R_array_[2] * m_tf.R_array_[7];
    m_R[2] = R_array_[0] * m_tf.R_array_[2] + R_array_[1] * m_tf.R_array_[5] + R_array_[2] * m_tf.R_array_[8];

    m_R[3] = R_array_[3] * m_tf.R_array_[0] + R_array_[4] * m_tf.R_array_[3] + R_array_[5] * m_tf.R_array_[6];
    m_R[4] = R_array_[3] * m_tf.R_array_[1] + R_array_[4] * m_tf.R_array_[4] + R_array_[5] * m_tf.R_array_[7];
    m_R[5] = R_array_[3] * m_tf.R_array_[2] + R_array_[4] * m_tf.R_array_[5] + R_array_[5] * m_tf.R_array_[8];

    m_R[6] = R_array_[6] * m_tf.R_array_[0] + R_array_[7] * m_tf.R_array_[3] + R_array_[8] * m_tf.R_array_[6];
    m_R[7] = R_array_[6] * m_tf.R_array_[1] + R_array_[7] * m_tf.R_array_[4] + R_array_[8] * m_tf.R_array_[7];
    m_R[8] = R_array_[6] * m_tf.R_array_[2] + R_array_[7] * m_tf.R_array_[5] + R_array_[8] * m_tf.R_array_[8];

    m_t[0] =
        R_array_[0] * m_tf.t_array_[0] + R_array_[1] * m_tf.t_array_[1] + R_array_[2] * m_tf.t_array_[2] + t_array_[0];
    m_t[1] =
        R_array_[3] * m_tf.t_array_[0] + R_array_[4] * m_tf.t_array_[1] + R_array_[5] * m_tf.t_array_[2] + t_array_[1];
    m_t[2] =
        R_array_[6] * m_tf.t_array_[0] + R_array_[7] * m_tf.t_array_[1] + R_array_[8] * m_tf.t_array_[2] + t_array_[2];

    Tf out_tf(m_R, m_t);
    return out_tf;
  }

  static Tf unitary()
  {
    return Tf(std::array<float, 9>{ 1, 0, 0, 0, 1, 0, 0, 0, 1 }, std::array<float, 3>{ 0, 0, 0 });
  }

  std::array<float, 9> R_array_{};
  std::array<float, 3> t_array_{};
};

}  // namespace vineslam
