#include "GIGI/FWBW.hxx"
#include "GIGI/gg_utils.hxx"

#define DEBUG_FB 0

#include <iostream>

using namespace GG;

constexpr size_t DEFAULT_SIZE{100};

// --------------------------------------------------------------------------------------------

// FWBW::FWBW(
//   const std::function<real(real, real)> &gg_Upper,
//   const std::function<real(real, real)> &gg_Lower,
//   const gg_range_max_min &gg_range
// )
//     : gg_Upper(gg_Upper), gg_Lower(gg_Lower), gg_range(gg_range)
// {
//   this->Segments.reserve(DEFAULT_SIZE);
//   this->Vmax_vec.reserve(DEFAULT_SIZE);
//   this->dump_seg_id.reserve(DEFAULT_SIZE);
// }
// IY : extend gg_Upper/gg_Lower with g argument; reserve G_vec
// HJ : add gg_Exp for Vmax mu correction
// IY : extend lambda signatures with slope argument for 3D ggv slope-aware lookup (option A)
FWBW::FWBW(
  const std::function<real(real, real, real, real)> &gg_Upper,
  const std::function<real(real, real, real, real)> &gg_Lower,
  const gg_range_max_min &gg_range,
  const std::function<real(real, real, real)> &gg_Exp
)
    : gg_Upper(gg_Upper), gg_Lower(gg_Lower), gg_range(gg_range), gg_Exp(gg_Exp)
{
  this->Segments.reserve(DEFAULT_SIZE);
  this->Vmax_vec.reserve(DEFAULT_SIZE);
  this->G_vec.reserve(DEFAULT_SIZE);
  this->MU_vec.reserve(DEFAULT_SIZE);   // HJ : slope angle vector
  this->DMU_vec.reserve(DEFAULT_SIZE);  // HJ : slope rate vector
  this->dump_seg_id.reserve(DEFAULT_SIZE);
}
// IY : end

// --------------------------------------------------------------------------------------------

// bool FWBW::is_in_range(const real ax, const real ay, const real v) const
// {
//   // Check if ax is within the upper or lower bounds
//   return ax <= this->gg_Upper(ay, v) && ax >= this->gg_Lower(ay, v) && ay >= this->gg_range.min(v)
//          && ay <= this->gg_range.max(v);
// }
// IY : add g argument and pass through to all bound functions
// IY : add slope argument for 3D ggv slope-aware lookup (option A)
bool FWBW::is_in_range(const real ax, const real ay, const real v, const real g, const real slope) const
{
  // Check if ax is within the upper or lower bounds
  return ax <= this->gg_Upper(ay, v, g, slope) && ax >= this->gg_Lower(ay, v, g, slope)
         && ay >= this->gg_range.min(v, g, slope) && ay <= this->gg_range.max(v, g, slope);
}
// IY : end

// --------------------------------------------------------------------------------------------

// real FWBW::signed_distance(const real ax, const real ay, const real v) const
// {
//   real ayclip = clip(ay, this->gg_range.min(v), this->gg_range.max(v));
//   return GG::signed_distance(
//     ax,
//     this->gg_Lower(ayclip, v),
//     this->gg_Upper(ayclip, v),
//     ay,
//     this->gg_range.min(v),
//     this->gg_range.max(v)
//   );
// }
// IY : add g argument; underlying GG::signed_distance utility is unchanged
// IY : add slope argument for 3D ggv slope-aware lookup (option A)
real FWBW::signed_distance(const real ax, const real ay, const real v, const real g, const real slope) const
{
  real ayclip = clip(ay, this->gg_range.min(v, g, slope), this->gg_range.max(v, g, slope));
  return GG::signed_distance(
    ax,
    this->gg_Lower(ayclip, v, g, slope),
    this->gg_Upper(ayclip, v, g, slope),
    ay,
    this->gg_range.min(v, g, slope),
    this->gg_range.max(v, g, slope)
  );
}
// IY : end

// --------------------------------------------------------------------------------------------

void FWBW::compute_Vmax()
{
  // extimate a small value of curvature.
  constexpr real k_small     = 1e-4;
  constexpr real v_top_speed = 130;
  real vmax                  = QUIET_NAN;
  // chose bracketing interval
  constexpr real v_b = v_top_speed;
  // instantiate vector same size of SS
  this->Vmax_vec.clear();
  this->Vmax_vec.resize(this->S_vec.size());
  // fill the vector
  for (int i = 0; i < this->S_vec.size(); i++)
  {
    constexpr real v_a = 0;
    real k = this->K_vec[i];
    // IY : capture per-node apparent gravity
    real g = this->G_vec[i];
    // IY : end
    // IY : per-node slope angle — used as both slope argument for 3D ggv lookup
    //      and gravity correction (HJ's mu_i)
    const real slope = this->MU_vec[i];
    // IY : end
    if (std::abs(k) < k_small)
    {
      vmax = v_top_speed;
    }
    else if (k >= k_small)
    {
      // IY : pass g and slope into gg_range.max
      auto F2solve = [this, k, g, slope](const real v) -> real { return k * v * v - this->gg_range.max(v, g, slope); };
      // IY : end
      bool ok = this->BD.solve(F2solve, v_a, v_b, vmax);
      // HJ : mu 보정 — 중력이 ax_tilde로 소모, diamond에서 ay 한계 감소
      //       (|ax_grav|/ax_max)^p + (|ay|/ay_max)^p <= 1
      //       → ay_max_eff = ay_max * (1 - (|ax_grav|/ax_max)^p)^(1/p)
      const real ax_grav = std::abs(9.81 * std::sin(slope));
      if (ax_grav > 1e-6 && !std::isnan(vmax) && this->gg_Exp) {
        const real ay_max_raw = this->gg_range.max(vmax, g, slope);
        const real ax_max_raw = this->gg_Upper(0.0, vmax, g, slope);  // ay=0일 때 ax_max
        const real p = this->gg_Exp(vmax, g, slope);
        if (ax_max_raw > 1e-6) {
          const real ratio = std::min(ax_grav / ax_max_raw, 1.0);
          const real ay_max_eff = ay_max_raw * std::pow(std::max(1.0 - std::pow(ratio, p), 0.0), 1.0 / p);
          const real vmax_corrected = std::sqrt(std::max(ay_max_eff, 0.01) / std::abs(k));
          vmax = std::min(vmax, vmax_corrected);
        }
      }
    }
    else
    {
      // IY : pass g and slope into gg_range.min
      auto F2solve = [this, k, g, slope](const real v) -> real { return k * v * v - this->gg_range.min(v, g, slope); };
      // IY : end
      bool ok = this->BD.solve(F2solve, v_a, v_b, vmax);
      const real ax_grav = std::abs(9.81 * std::sin(slope));
      if (ax_grav > 1e-6 && !std::isnan(vmax) && this->gg_Exp) {
        const real ay_min_raw = std::abs(this->gg_range.min(vmax, g, slope));
        const real ax_min_raw = std::abs(this->gg_Lower(0.0, vmax, g, slope));
        const real p = this->gg_Exp(vmax, g, slope);
        if (ax_min_raw > 1e-6) {
          const real ratio = std::min(ax_grav / ax_min_raw, 1.0);
          const real ay_min_eff = ay_min_raw * std::pow(std::max(1.0 - std::pow(ratio, p), 0.0), 1.0 / p);
          const real vmax_corrected = std::sqrt(std::max(ay_min_eff, 0.01) / std::abs(k));
          vmax = std::min(vmax, vmax_corrected);
        }
      }
    }
    vmax              = std::isnan(vmax) ? v_top_speed : vmax;
    // HJ : g_tilde > 0 constraint → v < sqrt(g*cos(mu) / dmu_ds) at crests
    //       prevents vehicle from going airborne on slope transitions
    const real dmu = this->DMU_vec[i];
    if (dmu > 1e-4) {
      const real v_gtilde_max = std::sqrt(9.81 * std::cos(slope) / dmu);
      vmax = std::min(vmax, v_gtilde_max);
    }
    this->Vmax_vec[i] = vmax;
  }
}

// --------------------------------------------------------------------------------------------

// real FWBW::compute(std::vector<real> const &SS, std::vector<real> const &KK, const real v0)
// {
//   this->S_vec = SS;
//   this->K_vec = KK;
//   this->v_I   = v0;
//   this->dump_seg_id.clear();
//   this->compute_Vmax();
//   this->FW();
//   this->BW();
//   return this->compute_time();
// }
// IY : add G_vec input (apparent gravity per node) for 3D track support
// HJ : add MU_vec input (slope angle per node) for ax_tilde correction
real FWBW::compute(
  std::vector<real> const &SS,
  std::vector<real> const &KK,
  std::vector<real> const &GG_,
  std::vector<real> const &MU_,
  std::vector<real> const &DMU_,
  const real v0)
{
  this->S_vec = SS;
  this->K_vec = KK;
  this->G_vec = GG_;
  this->MU_vec = MU_;
  this->DMU_vec = DMU_;
  this->v_I   = v0;
  this->dump_seg_id.clear();
  this->compute_Vmax();
  this->FW();
  this->BW();
  return this->compute_time();
}
// IY+HJ : end


// --------------------------------------------------------------------------------------------

real FWBW::compute_time() const
{
  real T = 0;
  for (const auto &seg : this->Segments)
  {
    T += seg.T();
  }
  return T;
}

// --------------------------------------------------------------------------------------------

// IY : forward pass — extract per-segment apparent gravity g0 = G_vec[i]
//      and pass it to all gg_Upper/gg_Lower/gg_range/signed_distance calls.
//      BD.solve lambda also captures g0.
// HJ : add ax_tilde correction: ax_tilde = ax - g*sin(mu)
//      GGV constraint applies to ax_tilde, so we shift ax limits by +g*sin(mu)
//      and pass ax_tilde to signed_distance.
void FWBW::FW()
{
  real v0 = this->v_I;
  this->Vmax_vec[0] = v0;
  this->Segments.clear();
  this->Segments.resize(this->S_vec.size() - 1);
  for (int i = 0; i < this->S_vec.size() - 1; i++)
  {
    // extract data for convenience of notation
    const real k0 = this->K_vec[i];
    const real k1 = this->K_vec[i + 1];
    const real g0 = this->G_vec[i];   // IY : per-segment apparent gravity (start node)
    const real mu0 = this->MU_vec[i]; // HJ : per-segment slope angle (also slope arg for 3D ggv lookup — IY option A)
    const real ax_gravity = 9.81 * std::sin(mu0); // HJ : longitudinal gravity component
    const real S0 = this->S_vec[i];
    const real S1 = this->S_vec[i + 1];
    const real L0 = S1 - S0;
    this->Segments[i] = segment(S0, L0, v0, k0, k1);
    const real ay0     = this->Segments[i].AY0();
    const real ay0clip = clip(ay0, this->gg_range.min(v0, g0, mu0), this->gg_range.max(v0, g0, mu0));
    // HJ : shift ax limits by ax_gravity (GGV gives tilde limits, we need real limits)
    const real axmax0  = this->gg_Upper(ay0clip, v0, g0, mu0) + ax_gravity;
    const real axmin0  = this->gg_Lower(ay0clip, v0, g0, mu0) + ax_gravity;
    // HJ : signed_distance checks ax_tilde = ax - ax_gravity against GGV diamond
    const real distance_amax  = this->signed_distance( axmax0 - ax_gravity, this->Segments[i].AYA(axmax0), this->Segments[i].VA(axmax0), g0, mu0);
    const real distance_azero = this->signed_distance(0.0 - ax_gravity, this->Segments[i].AYA(0.0), this->Segments[i].VA(0.0), g0, mu0);
    bool ok = false;
    real ax0 = 0.0;
    if (distance_amax <= 0)
    {
      ax0 = axmax0;
      ok  = true;
    }
    else
    {
      const real a_solver = std::min(
        this->gg_Lower(this->gg_range.min(v0, g0, mu0), v0, g0, mu0),
        this->gg_Lower(this->gg_range.max(v0, g0, mu0), v0, g0, mu0)
      ) + ax_gravity;  // HJ : shift solver bound
      ok = this->BD.solve(
        [this, &i, &v0, &g0, &mu0, &ax_gravity](const real ax) -> real
        { return this->signed_distance(ax - ax_gravity, this->Segments[i].AYA(ax), this->Segments[i].VA(ax), g0, mu0); },
        a_solver,
        axmax0,
        ax0
      );
    }
    if(ok)
    {
      this->Segments[i].set_a(ax0);
      this->Segments[i].set_type(FORWARD);
      v0 = std::max(0.0, std::min(this->Vmax_vec[i + 1], this->Segments[i].VF()));
      this->Vmax_vec[i + 1] = v0;
    }
    else
    {
      this->Segments[i].set_a(QUIET_NAN);
      this->Segments[i].set_type(FORWARD_NAN);
      v0 = std::max(0.0, std::min(this->Vmax_vec[i + 1], v0));
      this->Vmax_vec[i + 1] = v0;
    }
  }
}
// IY : end

// --------------------------------------------------------------------------------------------

// IY : backward pass — extract per-segment apparent gravity g1 = G_vec[i+1]
//      (segment end node) and pass it to all gg_* and signed_distance calls.
//      BD.solve lambda also captures g1.
// HJ : add ax_tilde correction with mu (same as FW pass)
void FWBW::BW()
{
  real v1 = this->Vmax_vec.back();
  if (this->Segments.back().type() == FORWARD)
  {
    v1 = this->Segments.back().VF();
  }
  // explore in reverse order
  for (auto i = static_cast<integer>(this->Segments.size() - 1); i >= 0; i--)
  {
    const real v0  = this->Segments[i].v0();
    const real kf  = this->Segments[i].k1();
    const real g1  = this->G_vec[i + 1];   // IY : per-segment apparent gravity (end node)
    const real mu1 = this->MU_vec[i + 1];  // HJ : per-segment slope angle (end node, also slope arg for 3D ggv lookup — IY option A)
    const real ax_gravity = 9.81 * std::sin(mu1); // HJ : longitudinal gravity component
    // HJ : v1을 다음 노드의 Vmax로 clamp — BW 전파 시 비정상 고속 방지
    v1 = std::min(v1, this->Vmax_vec[i + 1]);
    const real ayf = kf * v1 * v1;
    const real ayfclip = clip(ayf, this->gg_range.min(v1, g1, mu1), this->gg_range.max(v1, g1, mu1));
    // HJ : shift ax limits by ax_gravity
    const real axmaxf  = this->gg_Upper(ayfclip, v1, g1, mu1) + ax_gravity;
    const real axminf  = this->gg_Lower(ayfclip, v1, g1, mu1) + ax_gravity;
    const real amean = (v1 * v1 - v0 * v0) / (2 * this->Segments[i].L());
    const real v0_reach_max = std::min(this->Vmax_vec[i], this->Segments[i].VB(axminf, v1));
    const real v0_reach_min = std::max(0.0, this->Segments[i].VB(axmaxf, v1));
    const bool is_v0_reachable = (v0 >= v0_reach_min && v0 <= v0_reach_max);
    const bool is_amean_candidate = (amean >= axminf && amean <= axmaxf);
    // HJ : signed_distance checks ax_tilde = ax - ax_gravity
    const bool is_amean_valid = (this->signed_distance(amean - ax_gravity, this->Segments[i].AYB(amean, v1), this->Segments[i].VB(amean, v1), g1, mu1) <= this->solver_p.tolerance);
    const bool is_valid_forward = ((this->Segments[i].type() == FORWARD) &&  (std::abs(this->Segments[i].VF() - v1) <= this->solver_p.tolerance));
    if(is_valid_forward)
    {
      v1 = v0;
      continue;
    }
    if (is_amean_candidate && is_v0_reachable && is_amean_valid )
    {
      this->Segments[i].set_a(amean);
      this->Segments[i].set_type(TRANSITION);
      v1 = this->Segments[i].VB(amean, v1);
      continue;
    }
    else
    {
      // HJ : signed_distance with ax_tilde = ax - ax_gravity
      real distance_amax = this->signed_distance(axmaxf - ax_gravity, this->Segments[i].AYB(axmaxf, v1), this->Segments[i].VB(axmaxf, v1), g1, mu1);
      real distance_azero = this->signed_distance(0.0 - ax_gravity, this->Segments[i].AYB(0.0, v1), this->Segments[i].VB(0.0, v1), g1, mu1);
      real distance_amin = this->signed_distance(axminf - ax_gravity, this->Segments[i].AYB(axminf, v1), this->Segments[i].VB(axminf, v1), g1, mu1);
      //
      real ax0 = 0.0;
      bool ok = false;
      if (distance_amin <= 0)
      {
        ax0 = axminf;
        ok = true;
      }
      else
      {
        const real a_solver = std::max(
          this->gg_Upper(this->gg_range.min(v1, g1, mu1), v1, g1, mu1),
          this->gg_Upper(this->gg_range.max(v1, g1, mu1), v1, g1, mu1)
        ) + ax_gravity;  // HJ : shift solver bound
        ok = this->BD.solve(
          [this, &i, &v1, &g1, &mu1, &ax_gravity](const real ax) -> real {
            return this->signed_distance(ax - ax_gravity, this->Segments[i].AYB(ax, v1), this->Segments[i].VB(ax, v1), g1, mu1);
          },
          axminf,
          a_solver,
          ax0
        );
      }
      if (ok)
      {
        this->Segments[i].set_a(ax0);
        this->Segments[i].set_v0(this->Segments[i].VB(ax0, v1));
        this->Segments[i].set_type(BACKWARD);
        v1 = this->Segments[i].VB(ax0, v1);
      }
      else
      {
        // HJ : debug logging for NaN segments
        std::cout << "FWBW::BW() >> No solution found for segment " << i << "\n";
        std::cout << "  v0=" << v0 << " v1=" << v1 << " g1=" << g1 << " mu1=" << mu1 << "\n";
        std::cout << "  ax_gravity=" << ax_gravity << " axmaxf=" << axmaxf << " axminf=" << axminf << "\n";
        std::cout << "  amean=" << amean << " ayf=" << ayf << " ayfclip=" << ayfclip << "\n";
        std::cout << "  Vmax[i]=" << this->Vmax_vec[i] << " v0_reach_min=" << v0_reach_min << " v0_reach_max=" << v0_reach_max << "\n";
        std::cout << "  is_v0_reachable=" << is_v0_reachable << " is_amean_candidate=" << is_amean_candidate << " is_amean_valid=" << is_amean_valid << "\n";
        std::cout << "  distance_amax=" << distance_amax << " distance_amin=" << distance_amin << " distance_azero=" << distance_azero << "\n";
        this->dump_seg_id.push_back(i);
        this->Segments[i].set_a(QUIET_NAN);
        this->Segments[i].set_type(BACKWARD_NAN);
        // HJ : NaN 전파 방지 — FW 결과(v0)로 fallback하여 비정상 v1 전파 차단
        v1 = v0;
      }
    }
  }
}
// IY : end

// --------------------------------------------------------------------------------------------

void FWBW::evaluate(
  std::vector<real> const &SS, std::vector<real> &AX, std::vector<real> &AY, std::vector<real> &V
)
{
  for (integer i = 0; i < SS.size(); i++)
  {
    real s = SS[i];
    // Find the segment that contains s
    auto segment_it = std::find_if(
      this->Segments.begin(),
      this->Segments.end(),
      [s](const segment &seg) { return s >= seg.s0() && s <= seg.s1(); }
    );
    if (segment_it != this->Segments.end())
    {
      // Populate AX, AY, and V using the found segment
      AX[i] = segment_it->AX(s);
      AY[i] = segment_it->AY(s);
      V[i]  = segment_it->V(s);
    }
    else
    {
      // Handle the case where no segment is found
      std::cerr << "Error: No segment found for s = " << s << "\n";
      AX[i] = AY[i] = V[i] = 0.0;
    }
  }
}

// --------------------------------------------------------------------------------------------

integer FWBW::get_seg_idx(const real s) const
{
  integer seg_idx = -1;
  for (integer i = 0; i < this->Segments.size(); i++)
  {
    if (s >= this->Segments[i].s0() && s <= this->Segments[i].s1())
    {
      seg_idx = i;
      break;
    }
  }
  if (seg_idx == -1)
  {
    std::cerr << "Error: No segment found for s = " << s << "\n";
    return -1;
  }
  return seg_idx;
}

// --------------------------------------------------------------------------------------------

real FWBW::evalV(const real s) const
{
  const integer seg_idx = this->get_seg_idx(s);
  return this->Segments[seg_idx].V(s - this->Segments[seg_idx].s0());
}

// --------------------------------------------------------------------------------------------

real FWBW::evalAx(const real s) const
{
  const integer seg_idx = this->get_seg_idx(s);
  return this->Segments[seg_idx].AX(s - this->Segments[seg_idx].s0());
}

// --------------------------------------------------------------------------------------------

real FWBW::evalAy(const real s) const
{
  const integer seg_idx = this->get_seg_idx(s);
  return this->Segments[seg_idx].AY(s - this->Segments[seg_idx].s0());
}

// --------------------------------------------------------------------------------------------

real FWBW::evalVmax(const real s) const
{
  const integer seg_idx = this->get_seg_idx(s);
  const real s_norm = (s - this->Segments[seg_idx].s0()) / this->Segments[seg_idx].L();
  return this->Vmax_vec[seg_idx] * (1-s_norm) + this->Vmax_vec[seg_idx + 1] * s_norm;
}

// --------------------------------------------------------------------------------------------
