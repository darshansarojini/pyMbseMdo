import numpy as np

import csdl
import python_csdl_backend
import csdl_om
from modopt.scipy_library import SLSQP
from modopt.csdl_library import CSDLProblem
from lsdo_atmos.atmosphere_model import AtmosphereModel


class AircraftLevelSizing(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='g', default=9.81, types=float)
        return

    def define(self):
        Aeff = self.create_input(name='Aeff', desc='Aspect ratio')
        M = self.create_input(name='M', desc='Mach number')
        Phi25 = self.create_input(name='Phi25', units='deg', desc='Sweep angle, at 25% of chord')
        R = self.create_input(name='R', desc='Range')
        sfc_cr = self.create_input(name='sfc_cr', desc='Cruise SFC')
        BPR = self.create_input(name='BPR', desc='By-pass ratio')

        mLbymTO = self.create_input(name='mLbymTO', desc='Mass ratio, landing - take-off')
        mOEbymTO = self.create_input(name='mOEbymTO', desc='Relative operating empty mass')

        # region Constraint Analysis
        self.add(submodel=ConstraintAnalysis(),
                 name='ConstraintAnalysis', promotes=[])
        self.connect('Aeff', 'ConstraintAnalysis.Aeff')
        self.connect('Phi25', 'ConstraintAnalysis.Phi25')
        self.connect('mLbymTO', 'ConstraintAnalysis.mLbymTO')
        # endregion

        # region Cruise Aerodynamics
        self.add(submodel=CruiseAerodynamics(), name='CruiseAerodynamics', promotes=[])
        # endregion

        # Mission Analysis
        self.add(submodel=MissionAnalysis(), name='MissionAnalysis', promotes=[])

        self.connect('M', 'MissionAnalysis.M')
        self.connect('R', 'MissionAnalysis.R')
        self.connect('sfc_cr', 'MissionAnalysis.sfc_cr')
        self.connect('BPR', 'MissionAnalysis.BPR')
        self.connect('mLbymTO', 'MissionAnalysis.mLbymTO')
        self.connect('mOEbymTO', 'MissionAnalysis.mOEbymTO')

        self.connect('ConstraintAnalysis.TbyW', 'MissionAnalysis.TbyW')
        self.connect('CruiseAerodynamics.Emax', 'MissionAnalysis.Emax')
        # endregion

        # region Aircraft parameters
        WbyS = self.declare_variable(name='WbyS')
        self.connect('ConstraintAnalysis.WbyS', 'WbyS')
        TbyW = self.declare_variable(name='TbyW')
        self.connect('ConstraintAnalysis.TbyW', 'TbyW')
        m_mto = self.declare_variable(name='m_mto')
        self.connect('MissionAnalysis.m_mto', 'm_mto')

        Sw = m_mto / WbyS
        self.register_output(name='Sw', var=Sw)
        Tto = m_mto * self.parameters['g'] * TbyW
        self.register_output(name='Tto', var=Tto)
        # endregion
        return


class Landing(csdl.Model):
    def initialize(self):
        # Experience based values
        self.parameters.declare(name='kapp', default=1.7, types=float)
        # Constants
        self.parameters.declare(name='kts_to_ms', default=1.944, types=float)
        self.parameters.declare(name='g', default=9.81, types=float)
        return

    def define(self):
        slfl = self.declare_variable(name='slfl', units='m', desc='Landing field length')
        dTl = self.declare_variable(name='dTl', units='K', desc='Temperature above ISA (288,15K)')
        CLmax_L_unswept = self.declare_variable(name='CLmax_L_unswept',
                                                desc='Max. lift coefficient, landing for unswept wing')
        Phi25 = self.declare_variable(name='Phi25', units='deg', desc='Sweep angle, at 25% of chord')

        kapp = self.parameters['kapp']
        ktms = self.parameters['kts_to_ms']

        # Approach speed
        Vapp = kapp * slfl ** 0.5
        self.register_output(name='Vapp', var=Vapp)  # m/s
        Vapp = Vapp * ktms  # knots

        # Relative density
        s = 288.15/(288.15+dTl)

        # Landing Factor
        g = self.parameters['g']
        kL = (1.225/(2*g*1.3*1.3)) * kapp**2

        # Max lift coefficient at landing accounting for sweep
        CLmax_L_swept = CLmax_L_unswept * csdl.cos(np.pi/180.*Phi25)

        # Wing loading at max landing mass
        mLbySW = kL * s * CLmax_L_swept * slfl
        self.register_output(name='mLbySW', var=mLbySW)
        return


class Takeoff(csdl.Model):
    def initialize(self):
        # Experience based values
        self.parameters.declare(name='kto', default=2.34, types=float)  # m^3/kg
        return
    def define(self):
        stofl = self.declare_variable(name='stofl', units='m', desc='Take off field length')
        dTto = self.declare_variable(name='dTto', units='K', desc='Temperature above ISA (288,15K)')
        CLmax_TO_unswept = self.declare_variable(name='CLmax_TO_unswept',
                                                 desc='Max. lift coefficient, takeoff for unswept wing')
        Phi25 = self.declare_variable(name='Phi25', units='deg', desc='Sweep angle, at 25% of chord')
        mLbymTO = self.declare_variable(name='mLbymTO', desc='Mass ratio, landing - take-off')

        mLbySW = self.declare_variable(name='mLbySW', desc='Wing loading at max. landing mass')

        # Relative density
        s = 288.15 / (288.15 + dTto)

        # Max lift coefficient at takeoff accounting for sweep
        CLmax_TO_swept = CLmax_TO_unswept * csdl.cos(np.pi / 180. * Phi25)
        self.register_output(name='CLmax_TO_swept', var=CLmax_TO_swept)

        # Wing loading at max takeoff mass
        mTObySW = mLbySW/mLbymTO
        self.register_output(name='mTObySW', var=mTObySW)

        # Slope
        a = self.parameters['kto'] / (stofl * s * CLmax_TO_swept)

        # Thrust-to-weight ratio
        TtobyWto = mTObySW * a
        self.register_output(name='TtobyWto', var=TtobyWto)
        return


class GlideRatio(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='Lift-independent drag coefficient, clean', default=0.0188)  # todo: compute
        self.parameters.declare(name='Lift-independent drag coefficient, flaps', default=0.0347)  # todo: compute
        self.parameters.declare(name='Lift-independent drag coefficient, slats', default=0.)
        self.parameters.declare(name='Oswald efficiency factor; landing configuration', default=0.69)  # todo: compute
        self.parameters.declare(name='Climb gradient', default=0.024)
        return

    def define(self):
        Aeff = self.declare_variable(name='Aeff', desc='Aspect ratio')
        nE = self.declare_variable(name='nE', desc='Number of engines')

        # Lift coefficient at takeoff
        CLmax_TO_swept = self.declare_variable(name='CLmax_TO_swept',
                                               desc='Max lift coefficient at takeoff accounting for sweep')
        CLTO = CLmax_TO_swept / (1.2**2)

        # Lift-independent drag coefficient
        CD0 = self.parameters['Lift-independent drag coefficient, clean']
        dCD0_flap = self.parameters['Lift-independent drag coefficient, flaps']
        dCD0_slat = self.parameters['Lift-independent drag coefficient, slats']

        # Profile drag
        CDP = CD0 + dCD0_flap + dCD0_slat

        # Ostwald Efficiency factor, landing configuration
        e = self.parameters['Oswald efficiency factor; landing configuration']

        # Glide ratio in takeoff configuration
        L_D_TO = CLTO / (CDP+CLTO**2/(np.pi*Aeff*e))

        # Climb gradient
        sinGamma = self.parameters['Climb gradient']

        # Thrust-to-weight ratio
        TtobyWto = nE / (nE-1) * (1/L_D_TO+sinGamma)
        self.register_output(name='TtobyWto', var=TtobyWto)

        # # Maximum glide ratio
        # kE = 14.2
        # SwetbySw = 6.27
        # Emax = kE * (Aeff/SwetbySw)**2
        # self.register_output(name='Emax', var=Emax)
        return


class MissedApproach(csdl.Model):

    def initialize(self):
        return

    def define(self):
        temp = self.declare_variable(name='temp', val=0.27853)
        self.register_output(name='TbyW_ma', var=temp*1)
        # todo: compute
        return


class CruiseMatching(csdl.Model):
    def initialize(self):
        return

    def define(self):
        temp = self.declare_variable(name='temp', val=0.30750)
        self.register_output(name='TbyW_cr', var=temp * 1)
        # todo: compute
        return


class ConstraintAnalysis(csdl.Model):
    def initialize(self):
        return

    def define(self):

        Aeff = self.create_input(name='Aeff', desc='Aspect ratio')
        Phi25 = self.create_input(name='Phi25', units='deg', desc='Sweep angle, at 25% of chord')

        mLbymTO = self.create_input(name='mLbymTO', desc='Mass ratio, landing - take-off')

        # region Point performance
        # Landing
        self.add(submodel=Landing(), name='Landing', promotes=[])
        self.connect('Phi25', 'Landing.Phi25')
        # Takeoff
        self.add(submodel=Takeoff(), name='Takeoff', promotes=[])
        self.connect('Phi25', 'Takeoff.Phi25')
        self.connect('Landing.mLbySW', 'Takeoff.mLbySW')
        self.connect('mLbymTO', 'Takeoff.mLbymTO')
        # Glide ratio
        self.add(submodel=GlideRatio(), name='GlideRatio', promotes=[])
        self.connect('Aeff', 'GlideRatio.Aeff')
        self.connect('Takeoff.CLmax_TO_swept', 'GlideRatio.CLmax_TO_swept')
        # Missed approach
        self.add(submodel=MissedApproach(), name='MissedApproach', promotes=[])
        # Cruise matching
        self.add(submodel=CruiseMatching(), name='CruiseMatching', promotes=[])
        # endregion

        # region Sizing point

        TbyW_glide = self.declare_variable(name='TbyW_glide')
        self.connect('GlideRatio.TtobyWto', 'TbyW_glide')
        TbyW_takeoff = self.declare_variable(name='TbyW_takeoff')
        self.connect('Takeoff.TtobyWto', 'TbyW_takeoff')
        TbyW_ma = self.declare_variable(name='TbyW_ma')
        self.connect('MissedApproach.TbyW_ma', 'TbyW_ma')
        TbyW_cr = self.declare_variable(name='TbyW_cr')
        self.connect('CruiseMatching.TbyW_cr', 'TbyW_cr')

        WbyS_takeoff = self.declare_variable(name='WbyS_takeoff')
        self.connect('Takeoff.mTObySW', 'WbyS_takeoff')
        WbyS_landing = self.declare_variable(name='WbyS_landing')
        self.connect('Landing.mLbySW', 'WbyS_landing')

        scaling_parameter_TbyW = 1e3
        TbyW = csdl.max(TbyW_takeoff*scaling_parameter_TbyW,
                        TbyW_glide*scaling_parameter_TbyW,
                        TbyW_ma*scaling_parameter_TbyW,
                        TbyW_cr*scaling_parameter_TbyW,
                        rho=20.)
        self.register_output(name='TbyW', var=TbyW/scaling_parameter_TbyW)
        self.print_var(var=TbyW/scaling_parameter_TbyW)

        WbyS = csdl.max(WbyS_takeoff, WbyS_landing)
        self.register_output(name='WbyS', var=WbyS)
        # endregion

        return


class CruiseAerodynamics(csdl.Model):
    def initialize(self):
        return

    def define(self):
        temp = self.declare_variable(name='temp', val=17.48)
        self.register_output(name='Emax', var=temp * 1)
        # todo: compute
        return


class MissionAnalysis(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='nm_to_m', default=1852., types=float)
        self.parameters.declare(name='g', default=9.81, types=float)

        self.parameters.declare(name='Loiter time', default=1800., types=float)  # 1800s for international, 2700s for domestic
        return

    def define(self):
        BPR = self.declare_variable(name='BPR', desc='By-pass ratio')
        VbyVmd = self.declare_variable(name='VbyVmd')
        Emax = self.declare_variable(name='Emax', desc='Maximum glide ratio in cruise')
        M = self.declare_variable(name='M', desc='Mach number')
        TbyW = self.declare_variable(name='TbyW', desc='Thrust loading')
        # WbyS = self.declare_variable(name='WbyS', desc='Wing loading')
        R = self.declare_variable(name='R', desc='Design range')  # nautical miles
        s_alt = self.declare_variable(name='s_alt', desc='Distance to alternate')  # nautical miles
        s_res = self.declare_variable(name='s_res', desc='Reserve flight distance')  # m
        sfc_cr = self.declare_variable(name='sfc_cr', desc='Cruise specific fuel consumption')  # kg/N/s
        mLbymTO = self.declare_variable(name='mLbymTO', desc='Mass ratio, landing - take-off')
        mOEbymTO = self.declare_variable(name='mOEbymTO', desc='Relative operating empty mass')

        # Glide ratio in cruise
        CLbyCLmd = 1 / VbyVmd**2
        E = Emax*2/(1/CLbyCLmd+CLbyCLmd)
        # Thrust ratio
        TR = 1/(TbyW*E)

        # Cruise altitude
        hcr = (TR - 0.7125 + 0.0248 * BPR) / (0.0013 * BPR - 0.0397) * 1000  # m
        self.register_output(name='hcr', var=hcr)

        # Cruise speed
        self.add(submodel=AtmosphereModel(shape=(1,)), name='Atmosisa', promotes=[])
        acr = self.declare_variable(name='acr')
        self.connect('hcr', 'Atmosisa.z')
        self.connect('Atmosisa.speed_of_sound', 'acr')
        # Tcr = 216.65
        # acr = 20.05 * Tcr**0.5
        Vcr = acr * M  # m/s
        self.register_output(name='Vcr', var=Vcr)

        # Mission ranges
        R = R * self.parameters['nm_to_m']
        # s_alt = s_alt * self.parameters['nm_to_m']

        # Cruise
        B_cruise = E * Vcr / sfc_cr / self.parameters['g']  # Breguet factor
        Mff_cr = csdl.exp(-R/B_cruise)  # Fuel fraction, cruise
        Mff_res = csdl.exp(-s_res/B_cruise)  # Fuel fraction, extra flight distance

        # Loiter
        t_loiter = self.parameters['Loiter time']  # s
        sfc_loiter = sfc_cr
        B_loiter = B_cruise/Vcr
        Mff_loiter = csdl.exp(-t_loiter/B_loiter)

        #  Emperical fuel fractions
        Mff_taxi = 0.997
        Mff_to = 0.993
        Mff_clb = 0.993
        Mff_des = 0.993
        Mff_l = 0.993

        # Final fuel fractions
        Mff_standard = Mff_to * Mff_clb * Mff_cr * Mff_des * Mff_l
        Mff_reserve = Mff_clb * Mff_res * Mff_loiter * Mff_des
        Mff_total = Mff_standard * Mff_reserve
        mFbymMTO = 1 - Mff_total  # Mission fuel fraction

        # Masses
        m_pax = 93  # kg
        n_pax = 180
        m_cargo = 2516  # kg
        m_pl = m_pax * n_pax + m_cargo
        m_mto = m_pl / (1-mFbymMTO-mOEbymTO)
        self.register_output(name='m_mto', var=m_mto)

        # Mission fuel, standard flight
        m_f = m_mto * mFbymMTO
        self.register_output(name='m_f', var=m_f)

        # Maximum landing mass, based on ratio
        m_ml = m_mto * mLbymTO
        self.register_output(name='m_ml', var=m_ml)

        # Operating empty mass
        m_oe = m_mto * mOEbymTO
        self.register_output(name='m_oe', var=m_oe)

        # Total fuel mass needed
        m_f_erf = m_mto * (1-Mff_taxi*Mff_total)
        self.register_output(name='m_f_erf', var=m_f_erf)

        # Maximum zero-fuel mass
        m_mzf = m_oe + m_pl
        self.register_output(name='m_mzf', var=m_mzf)

        # Fuel mass, all reserves
        m_f_res = m_mto * (1-Mff_reserve)

        # Maximum landing mass, computed by adding masses
        m_ml_comp = m_mzf + m_f_res

        # Constraint on maximum landing mass
        # m_ml > m_mzf + m_f_res
        constr_m_ml = m_ml - m_ml_comp
        self.register_output(name='Constraint_m_ml', var=constr_m_ml)
        self.add_constraint(name='Constraint_m_ml', lower=0.)
        self.print_var(var=constr_m_ml)
        return


class FuelTankVolume(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='FuelDensity', default=800., types=float)
        return

    def define(self):
        m_f_erf = self.declare_variable(name='m_f_erf', desc='Total fuel mass needed (kg)')
        rho_f = self.parameters['FuelDensity']
        V_f_erf = m_f_erf/rho_f

        return


if __name__ == '__main__':
    sim = python_csdl_backend.Simulator(AircraftLevelSizing())

    # Aircraft design variables
    sim['Phi25'] = 25.
    sim['Aeff'] = 9.5
    sim['BPR'] = 6.
    sim['ConstraintAnalysis.Landing.CLmax_L_unswept'] = 3.76
    sim['ConstraintAnalysis.Takeoff.CLmax_TO_unswept'] = 2.85
    # sim['MissionAnalysis.Emax'] = 17.48

    # Operating conditions
    sim['M'] = 0.76
    sim['R'] = 1510.  # NM
    sim['ConstraintAnalysis.Landing.slfl'] = 1448.
    sim['ConstraintAnalysis.Landing.dTl'] = 0.
    sim['ConstraintAnalysis.Takeoff.stofl'] = 1768.
    sim['ConstraintAnalysis.Takeoff.dTto'] = 0.
    sim['ConstraintAnalysis.GlideRatio.nE'] = 2

    # Emperical values
    sim['mLbymTO'] = 0.878
    sim['mOEbymTO'] = 0.56
    sim['MissionAnalysis.s_res'] = 510226.  # m
    # sim['MissionAnalysis.s_alt'] = 200.  # NM

    sim['sfc_cr'] = 1.65E-05  # kg/N/s

    sim['MissionAnalysis.VbyVmd'] = 0.94844796  # 0.94844796

    sim.run()
    print('Approach speed (m/s): ', sim['ConstraintAnalysis.Landing.Vapp'])
    print('Wing loading at max. landing mass: ', sim['ConstraintAnalysis.Landing.mLbySW'])
    print('Wing loading at max. takeoff mass: ', sim['ConstraintAnalysis.Takeoff.mTObySW'])
    print('Thrust-to-weight ratio takeoff: ', sim['ConstraintAnalysis.Takeoff.TtobyWto'])
    print('Thrust-to-weight ratio glide: ', sim['ConstraintAnalysis.GlideRatio.TtobyWto'])
    print('------------------\n------------------')
    print('Sizing thrust-to-weight ratio: ', sim['ConstraintAnalysis.TbyW'])
    print('Sizing wing loading: ', sim['ConstraintAnalysis.WbyS'])
    print('------------------\n------------------')
    print('Maximum takeoff mass (kg) ', sim['m_mto'])
    print('Maximum landing mass (kg) ', sim['MissionAnalysis.m_ml'])
    print('Mission fuel mass (kg) ', sim['MissionAnalysis.m_f'])
    print('Total fuel mass (kg) ', sim['MissionAnalysis.m_f_erf'])
    print('Operating empty mass (kg) ', sim['MissionAnalysis.m_oe'])
    print('Wing area (m^2) ', sim['Sw'])
    print('Takeoff thrust, all engines (N) ', sim['Tto'])
