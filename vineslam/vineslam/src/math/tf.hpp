#pragma once

#include <iostream>
#include <array>

namespace vineslam
{
struct TF {
  TF()
  {
    R = {1., 0., 0., 0., 1., 0., 0., 0., 1.};
    t = {0., 0., 0.};
  }

  TF(const std::array<float, 9>& m_R, const std::array<float, 3>& m_t)
  {
    R = m_R;
    t = m_t;
  }

  TF inverse()
  {
    std::array<float, 9> R_inverse = transposeRotation();
    std::array<float, 3> t_inverse{};
    t_inverse[0] =
        R_inverse[0] * (-t[0]) + R_inverse[1] * (-t[1]) + R_inverse[2] * (-t[2]);
    t_inverse[1] =
        R_inverse[3] * (-t[0]) + R_inverse[4] * (-t[1]) + R_inverse[5] * (-t[2]);
    t_inverse[2] =
        R_inverse[6] * (-t[0]) + R_inverse[7] * (-t[1]) + R_inverse[8] * (-t[2]);

    TF inv_tf(R_inverse, t_inverse);

    return inv_tf;
  }

  std::array<float, 9> transposeRotation()
  {
    std::array<float, 9> R_transpose = {
        R[0], R[3], R[6], R[1], R[4], R[7], R[2], R[5], R[8]};

    return R_transpose;
  }

  TF operator*(const TF& m_tf)
  {
    std::array<float, 9> m_R{};
    std::array<float, 3> m_t{};

    m_R[0] = R[0] * m_tf.R[0] + R[1] * m_tf.R[3] + R[2] * m_tf.R[6];
    m_R[1] = R[0] * m_tf.R[1] + R[1] * m_tf.R[4] + R[2] * m_tf.R[7];
    m_R[2] = R[0] * m_tf.R[2] + R[1] * m_tf.R[5] + R[2] * m_tf.R[8];

    m_R[3] = R[3] * m_tf.R[0] + R[4] * m_tf.R[3] + R[5] * m_tf.R[6];
    m_R[4] = R[3] * m_tf.R[1] + R[4] * m_tf.R[4] + R[5] * m_tf.R[7];
    m_R[5] = R[3] * m_tf.R[2] + R[4] * m_tf.R[5] + R[5] * m_tf.R[8];

    m_R[6] = R[6] * m_tf.R[0] + R[7] * m_tf.R[3] + R[8] * m_tf.R[6];
    m_R[7] = R[6] * m_tf.R[1] + R[7] * m_tf.R[4] + R[8] * m_tf.R[7];
    m_R[8] = R[6] * m_tf.R[2] + R[7] * m_tf.R[5] + R[8] * m_tf.R[8];

    m_t[0] = R[0] * m_tf.t[0] + R[1] * m_tf.t[1] + R[2] * m_tf.t[2] + t[0];
    m_t[1] = R[3] * m_tf.t[0] + R[4] * m_tf.t[1] + R[5] * m_tf.t[2] + t[1];
    m_t[2] = R[6] * m_tf.t[0] + R[7] * m_tf.t[1] + R[8] * m_tf.t[2] + t[2];

    TF out_tf(m_R, m_t);
    return out_tf;
  }

  std::array<float, 9> R{};
  std::array<float, 3> t{};
};

// stdout operator
static std::ostream& operator<<(std::ostream& out, TF const& tf)
{
  return out << tf.R[0] << " " << tf.R[1] << " " << tf.R[2] << " " << tf.t[0] << "\n"
             << tf.R[3] << " " << tf.R[4] << " " << tf.R[5] << " " << tf.t[1] << "\n"
             << tf.R[6] << " " << tf.R[7] << " " << tf.R[8] << " " << tf.t[2]
             << "\n";
}

} // namespace vineslam
