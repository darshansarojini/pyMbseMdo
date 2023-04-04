import numpy as np

import csdl
import python_csdl_backend
import csdl_om


class AircraftLevelSizing(csdl.Model):
    def initialize(self):
        return

    def define(self):
        Aeff = self.create_input(name='Aeff', desc='Aspect ratio')
        M = self.create_input(name='M', desc='Mach number')
        Phi25 = self.create_input(name='Phi25', units='deg', desc='Sweep angle, at 25% of chord')
        R = self.create_input(name='R', desc='Range')
        sfc_cr = self.create_input(name='sfc_cr', desc='Cruise SFC')
        BPR = self.create_input(name='BPR', desc='By-pass ratio')

        # region Constraint Analysis
        # Landing
        self.add(submodel=Landing(), name='Landing', promotes=[])
        self.connect('Phi25', 'Landing.Phi25')
        # Takeoff
        self.add(submodel=Takeoff(), name='Takeoff', promotes=[])
        self.connect('Phi25', 'Takeoff.Phi25')
        self.connect('Landing.mLbySW', 'Takeoff.mLbySW')
        # Glide ratio
        self.add(submodel=GlideRatio(), name='GlideRatio', promotes=[])
        self.connect('Aeff', 'GlideRatio.Aeff')
        self.connect('Takeoff.CLmax_TO_swept', 'GlideRatio.CLmax_TO_swept')
        # endregion

        # Mission Analysis
        self.add(submodel=MissionAnalysis(), name='MissionAnalysis', promotes=[])
        self.connect('M', 'MissionAnalysis.M')
        self.connect('R', 'MissionAnalysis.R')
        self.connect('sfc_cr', 'MissionAnalysis.sfc_cr')
        self.connect('BPR', 'MissionAnalysis.BPR')
        # endregion

        # # Aircraft parameters
        # Sw = m_mto / WbyS
        # Tto = m_mto * self.parameters['g'] * TbyW
        return


class Landing(csdl.Model):
    def initialize(self):
        # Experience based values
        self.parameters.declare(name='kapp', default=1.7, types=float)
        # Constants
        self.parameters.declare(name='kts_to_ms', default=1.944, types=float)
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
        g = 9.81
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

        return

    def define(self):
        Aeff = self.declare_variable(name='Aeff', desc='Aspect ratio')
        nE = self.declare_variable(name='nE', desc='Number of engines')

        # Lift coefficient at takeoff
        CLmax_TO_swept = self.declare_variable(name='CLmax_TO_swept',
                                               desc='Max lift coefficient at takeoff accounting for sweep')
        CLTO = CLmax_TO_swept / (1.2**2)

        # Lift-independent drag coefficient
        CD0 = 0.0188  # Clean
        dCD0_flap = 0.0347  # Flaps
        dCD0_slat = 0.  # Slats

        # Profile drag
        CDP = CD0 + dCD0_flap + dCD0_slat

        # Ostwald Efficiency factor, landing configuration
        e = 0.69

        # Glide ratio in takeoff configuration
        ETO = CLTO / (CDP+CLTO**2/(np.pi*Aeff*e))

        # Climb gradient
        sinGamma = 0.024

        # Thrust-to-weight ratio
        TtobyWto = nE / (nE-1) * (1/ETO+sinGamma)
        self.register_output(name='TtobyWto', var=TtobyWto)

        # Maximum glide ratio
        kE = 14.2
        SwetbySw = 6.27
        Emax = kE * (Aeff/SwetbySw)**2
        self.register_output(name='Emax', var=Emax)
        return


class MissionAnalysis(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='nm_to_m', default=1852., types=float)
        self.parameters.declare(name='g', default=9.81, types=float)
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

        # Glide ratio in cruise
        CLbyCLmd = 1 / VbyVmd**2
        E = Emax*2/(1/CLbyCLmd+CLbyCLmd)
        # Thrust ratio
        TR = 1/(TbyW*E)

        # Cruise altitude
        hcr = (TR - 0.7125 + 0.0248 * BPR) / (0.0013 * BPR - 0.0397) * 1000  # m
        self.register_output(name='hcr', var=hcr)

        # Cruise speed
        # Tcr = 288.15 - 0.0065 * hcr  # K
        # Tstrat = self.declare_variable(name='Tstrat', val=216.65)  # K
        # if Tcr < Tstrat:
        #     Tcr = Tstrat
        Tcr = 216.65
        acr = 20.05 * Tcr**0.5
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
        t_loiter = 1800  # s
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
        mFbymMTO = 1 - Mff_total

        # Masses
        mOEbymMTO = 0.56  # relative operating empty mass
        m_pax = 93  # kg
        n_pax = 180
        m_cargo = 2516  # kg
        m_pl = m_pax * n_pax + m_cargo
        m_mto = m_pl / (1-mFbymMTO-mOEbymMTO)
        self.register_output(name='m_mto', var=m_mto)
        m_f = m_mto * mFbymMTO
        self.register_output(name='m_f', var=m_f)
        return


if __name__ == '__main__':
    sim = python_csdl_backend.Simulator(AircraftLevelSizing())

    sim['Phi25'] = 25.
    sim['Aeff'] = 9.5
    sim['M'] = 0.76
    sim['R'] = 1510.  # NM
    sim['sfc_cr'] = 1.65E-05  # kg/N/s
    sim['BPR'] = 6.

    sim['Landing.slfl'] = 1448.
    sim['Landing.dTl'] = 0.

    sim['Landing.CLmax_L_unswept'] = 3.76

    sim['Takeoff.mLbymTO'] = 0.878
    sim['Takeoff.stofl'] = 1768.
    sim['Takeoff.dTto'] = 0.
    sim['Takeoff.CLmax_TO_unswept'] = 2.85

    sim['GlideRatio.nE'] = 2

    sim['MissionAnalysis.VbyVmd'] = 0.94844796  # 0.94844796
    sim['MissionAnalysis.Emax'] = 17.48
    sim['MissionAnalysis.TbyW'] = 0.30750
    # sim['MissionAnalysis.WbyS'] = 600.05
    sim['MissionAnalysis.s_res'] = 510226.  # m
    # sim['s_alt'] = 200.  # NM

    sim.run()
    print('Approach speed (m/s): ', sim['Landing.Vapp'])
    print('Wing loading at max. landing mass: ', sim['Landing.mLbySW'])
    print('Wing loading at max. takeoff mass: ', sim['Takeoff.mTObySW'])
    print('Thrust-to-weight ratio: ', sim['Takeoff.TtobyWto'])
    print('Thrust-to-weight ratio: ', sim['GlideRatio.TtobyWto'])
    print('Aircraft mass (kg) ', sim['MissionAnalysis.m_mto'])
