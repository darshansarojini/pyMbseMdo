import scipy.io as sio
import numpy as np
import sys
import csdl
import python_csdl_backend
import csdl_om
# from modopt.scipy_library import SLSQP
# from modopt.csdl_library import CSDLProblem
from modopt.external_libraries.csdl import CSDLProblem
from modopt.external_libraries.scipy import SLSQP
from lsdo_atmos.atmosphere_model import AtmosphereModel


class AircraftLevelSizing(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='g', default=9.81, types=float)
        return

    def define(self):

        # Sw_guess = self.create_input(name='Sw_guess', desc='Guess for wing area')
        # self.add_design_variable(dv_name='Sw_guess', lower=110., upper=140.)

        Aeff = self.create_input(name='Aeff', desc='Aspect ratio')
        M = self.create_input(name='M', desc='Mach number')
        Phi25 = self.create_input(name='Phi25', units='deg', desc='Sweep angle, at 25% of chord')
        R = self.create_input(name='R', desc='Range')
        BPR = self.create_input(name='BPR', desc='By-pass ratio')
        lambdaeff = self.create_input(name='lambdaeff', desc='Taper ratio')
        dFo = self.create_input(name='dFo', desc='Fuselage outer diameter')
        nE = self.create_input(name='nE', desc='Number of engines')
        tbyc = self.create_input(name='tbyc', desc='Wing relative thickness, average')

        mLbymTO = self.create_input(name='mLbymTO', desc='Mass ratio, landing - take-off')

        VbyVmd = self.create_input(name='VbyVmd')
        self.add_design_variable(dv_name='VbyVmd', lower=0.65, upper=1.75)

        # region Cruise Aerodynamics
        self.add(submodel=CruiseAerodynamics(),
                 name='CruiseAerodynamics', promotes=[])
        self.connect('Aeff', 'CruiseAerodynamics.Aeff')
        self.connect('lambdaeff', 'CruiseAerodynamics.lambdaeff')
        self.connect('Phi25', 'CruiseAerodynamics.Phi25')
        self.connect('dFo', 'CruiseAerodynamics.dFo')
        self.connect('Sw', 'CruiseAerodynamics.Sw')
        self.connect('M', 'CruiseAerodynamics.M')
        # endregion

        # region SFC Calculation
        self.add(submodel=SfcCalculation(),
                 name='SfcCalculation', promotes=[])
        self.connect('M', 'SfcCalculation.M')
        self.connect('BPR', 'SfcCalculation.BPR')
        self.connect('MissionAnalysis.hcr', 'SfcCalculation.hcr')
        self.connect('Tto', 'SfcCalculation.Tto')
        self.connect('nE', 'SfcCalculation.nE')
        # endregion

        # region Constraint Analysis
        self.add(submodel=ConstraintAnalysis(),
                 name='ConstraintAnalysis', promotes=[])
        self.connect('Aeff', 'ConstraintAnalysis.Aeff')
        self.connect('Phi25', 'ConstraintAnalysis.Phi25')
        self.connect('mLbymTO', 'ConstraintAnalysis.mLbymTO')
        self.connect('VbyVmd', 'ConstraintAnalysis.VbyVmd')
        self.connect('M', 'ConstraintAnalysis.M')
        self.connect('BPR', 'ConstraintAnalysis.BPR')
        self.connect('nE', 'ConstraintAnalysis.nE')
        self.connect('CruiseAerodynamics.Emax', 'ConstraintAnalysis.Emax')
        self.connect('CruiseAerodynamics.CD0', 'ConstraintAnalysis.CD0')
        self.connect('CruiseAerodynamics.e', 'ConstraintAnalysis.e')
        # endregion

        # region Mission Analysis
        self.add(submodel=MissionAnalysis(), name='MissionAnalysis', promotes=[])

        self.connect('M', 'MissionAnalysis.M')
        self.connect('R', 'MissionAnalysis.R')
        self.connect('BPR', 'MissionAnalysis.BPR')
        self.connect('mLbymTO', 'MissionAnalysis.mLbymTO')
        self.connect('VbyVmd', 'MissionAnalysis.VbyVmd')
        self.connect('Sw', 'MissionAnalysis.Sw')

        self.connect('ConstraintAnalysis.TbyW', 'MissionAnalysis.TbyW')
        self.connect('CruiseAerodynamics.Emax', 'MissionAnalysis.Emax')
        self.connect('ConstraintAnalysis.CruiseMatching.CL_cruise', 'MissionAnalysis.CL_cr')
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

        # Sw_constraint = Sw_guess - Sw
        # self.print_var(var=Sw_constraint)
        # self.register_output(name='Sw_constraint', var=Sw_constraint)
        # self.add_constraint(name='Sw_constraint', equals=0.)

        Tto = m_mto * self.parameters['g'] * TbyW
        self.register_output(name='Tto', var=Tto)
        # endregion

        # region Fuel Tank Volume Constraint
        self.add(submodel=FuelTankVolume(), promotes=[], name='FuelTankVolume')
        self.connect('Aeff', 'FuelTankVolume.Aeff')
        self.connect('lambdaeff', 'FuelTankVolume.lambdaeff')
        self.connect('Sw', 'FuelTankVolume.Sw')
        self.connect('tbyc', 'FuelTankVolume.tbyc')
        self.connect('MissionAnalysis.m_f_erf', 'FuelTankVolume.m_f_erf')
        # endregion

        return


# region Functions and Constraints
class FuelTankVolume(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='FuelDensity', default=800., types=float)
        self.parameters.declare(name='kCS', default=-2., desc='CS 25.979(b) fuel tank correction', types=float)  # %
        self.parameters.declare(name='kUF', default=-3., desc='Correction accounting for unusable fuel', types=float)  # %
        return

    def define(self):
        m_f_erf = self.declare_variable(name='m_f_erf', desc='Total fuel mass needed (kg)')
        Aeff = self.declare_variable(name='Aeff', desc='Aspect ratio')
        lambdaeff = self.declare_variable(name='lambdaeff', desc='Taper ratio')
        tbyc = self.declare_variable(name='tbyc', desc='Wing average relative thickness')
        Sw = self.declare_variable(name='Sw', desc='Wing area')

        # Fuel volume needed
        rho_f = self.parameters['FuelDensity']
        V_f_erf = m_f_erf / rho_f
        self.register_output(name='FuelVolumeNeeded', var=V_f_erf)

        # Tank volume available
        Vtank = 0.54 * Sw ** 1.5 * tbyc * (1 / Aeff**0.5) * (1 + lambdaeff + lambdaeff ** 2) / (1 + lambdaeff) ** 2
        Vtank = Vtank * (1 + (self.parameters['kCS']/100 + self.parameters['kUF']/100))
        self.register_output(name='FuelTankVolume', var=Vtank)

        # Fuel tank volume constraint
        # Vtank > V_f_erf
        constr_vtank = V_f_erf - Vtank
        self.register_output(name='Constraint_FuelTankVolume', var=constr_vtank)
        self.add_constraint(name='Constraint_FuelTankVolume', upper=0., scaler=1e-2)
        return


class SfcCalculation(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='dPbyp', default=2, types=int,
                                desc='Inlet pressure loss %')
        self.parameters.declare(name='T0', default=288.15, types=float,
                                desc='Temperature (ISA) at SL')  # K
        self.parameters.declare(name='L', default=0.0065, types=float,
                                desc='Temperature lapse rate in troposphere')  # K/m
        self.parameters.declare(name='TS', default=216.65, types=float,
                                desc='Temperature (ISA) at tropopause')  # K
        self.parameters.declare(name='gamma', default=1.4,
                                types=float, desc='Ratio of specific heats, air')
        return

    def define(self):
        M = self.declare_variable(name='M', desc='Mach number')
        BPR = self.declare_variable(name='BPR', desc='By-pass ratio')
        nE = self.declare_variable(name='nE', desc='Number of engines')

        hcr = self.declare_variable(name='hcr', desc='Cruise altitude')
        Tto = self.declare_variable(name='Tto', desc='Take-off thrust')

        # Thrust of a single engine
        T_oneengine = Tto/nE * 1e-3  # kN

        # Overall Pressure ratio
        OAPR = 0.0266785 * T_oneengine * 10 ** -3 + 3.5158 * BPR + 0.0556628

        # Turbine entry temperature
        T_TE = -8000 / T_oneengine + 1520

        # Inlet efficiency
        eta_inlet = 1 - (1.3 + 0.25*BPR) * (self.parameters['dPbyp']/100)

        # Fan efficiency
        eta_fan = -5.978/(5.978+T_oneengine)-M*0.1479-0.133498/(0.133498+BPR)+1.05489

        # Compressor efficiency
        eta_comp = -2/(2+T_oneengine)-0.1171127/(0.1171127+BPR)-M*0.0541+0.9407245

        # Turbine efficiency
        eta_turb = -3.403/(3.403+T_oneengine)+1.04826-M*0.15533

        # Nozzle efficiency
        eta_noz = -2.0319/(2.0319+T_oneengine)+1.00764-M*0.009868

        # Temperature at cruise altitude
        T = self.parameters['T0'] - self.parameters['L'] * hcr

        # Dimensionless turbine entry temperature
        phi = T_TE / T

        # Ratio between stagnation point temperature and ambient temperature
        nu = 1+(self.parameters['gamma']-1)/2*M**2

        # Temperature function
        xi = nu*(OAPR**((self.parameters['gamma']-1)/self.parameters['gamma'])-1)

        # Gas generator efficiency
        eta_gasgen = 1-(0.7*M**2*(1-eta_inlet))/(1+0.2*M**2)

        # Gas generator function
        G = (phi-xi/eta_comp) * \
            (1-1.01/(eta_gasgen**((self.parameters['gamma']-1)/self.parameters['gamma'])*(xi+nu)
                     * (1-xi/phi/eta_comp/eta_turb))
             )

        # SFC
        SFC = (0.697 * (T/self.parameters['T0'])**0.5 * (phi-nu-xi/eta_comp)) / \
              ((5*eta_noz * (1+eta_fan*eta_turb*BPR) *
                (G+0.2*M**2*BPR*eta_comp/eta_fan/eta_turb))**0.5 - M*(1+BPR))  # kg/daN/h
        SFC = SFC * 1/36000
        self.print_var(SFC)

        temp = self.declare_variable(name='temp', val=1.650E-05)
        self.register_output(name='SFC', var=temp*1)

        return


class CruiseAerodynamics(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='ke_D0', default=0.903,
                                desc='Factor for zero-lift drag effect')
        self.parameters.declare(name='Mcomp', default=0.3,
                                desc='Highest Mach number without compressibility effects')
        # Coefficients of equation
        self.parameters.declare(name='ae', default=-0.00270)
        self.parameters.declare(name='be', default=8.60)
        self.parameters.declare(name='ce', default=1.0)

        self.parameters.declare(name='Cfeqv', default=0.003,
                                desc='Equivalent surface friction coefficient')
        self.parameters.declare(name='SwetbySw', default=6.27,
                                desc='Relative wetted area')
        return

    def define(self):
        Aeff = self.declare_variable(name='Aeff', desc='Aspect ratio')
        lambdaeff = self.declare_variable(name='lambdaeff', desc='Taper ratio')
        Phi25 = self.declare_variable(name='Phi25', desc='Sweep angle, at 25% of chord')
        dFo = self.declare_variable(name='dFo', desc='Fuselage outer diameter')
        Sw = self.declare_variable(name='Sw', desc='Wing area')
        M = self.declare_variable(name='M', desc='Mach number')

        dlambda = -0.35659 + 0.45 * csdl.exp(-0.0375 * Phi25)

        # Corrected horner function
        horner = 0.0524 * (lambdaeff - dlambda) ** 4 - 0.15 * (lambdaeff - dlambda) ** 3 + \
                 0.1659 * (lambdaeff - dlambda) ** 2 - 0.0706 * (lambdaeff - dlambda) + \
                 0.0119

        # Ostwald factor, theoretical
        e_theo = 1 / (1 + horner * Aeff)

        # Geometrical span
        beff = (Aeff * Sw) ** 0.5  # m

        # Fuselage factor
        dfobybeff = dFo / beff
        ke_F = 1 - 2 * dfobybeff ** 2

        # Factor for compressibility effect
        MbyMcomp = M / self.parameters['Mcomp']
        ke_M = self.parameters['ae'] * (MbyMcomp - 1) ** self.parameters['be'] + self.parameters['ce']

        # Ostwald factor corrected
        e = e_theo * ke_F * self.parameters['ke_D0'] * ke_M

        # ke
        ke = 1 / 2 * (np.pi * e / self.parameters['Cfeqv']) ** 0.5

        # Maximum glide ratio
        Emax = ke * (Aeff / self.parameters['SwetbySw']) ** 0.5

        # Zero-lift drag coefficient
        CD0 = np.pi * Aeff * e / 4 / Emax ** 2
        self.print_var(CD0)

        temp = self.declare_variable(name='temp', val=17.48)
        self.register_output(name='Emax', var=temp * 1)

        temp1 = self.declare_variable(name='temp1', val=0.0188)
        self.register_output(name='CD0', var=temp1 * 1)

        temp2 = self.declare_variable(name='temp2', val=0.770)
        self.register_output(name='e', var=temp2 * 1)
        return
# endregion


# region Constraint Analysis
class ConstraintAnalysis(csdl.Model):
    def initialize(self):
        return

    def define(self):

        Aeff = self.create_input(name='Aeff', desc='Aspect ratio')
        Phi25 = self.create_input(name='Phi25', units='deg', desc='Sweep angle, at 25% of chord')

        mLbymTO = self.create_input(name='mLbymTO', desc='Mass ratio, landing - take-off')
        nE = self.create_input(name='nE', desc='Number of engines')
        VbyVmd = self.create_input(name='VbyVmd', desc='Design variable')
        M = self.create_input(name='M', desc='Mach number')
        BPR = self.create_input(name='BPR', desc='By-pass ratio')
        Emax = self.create_input(name='Emax', desc='Maximum glide ratio in cruise')
        CD0 = self.create_input(name='CD0', desc='Lift-independent drag coefficient, clean')
        e = self.create_input(name='e', desc='Oswald factor')

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
        self.connect('nE', 'GlideRatio.nE')
        self.connect('CD0', 'GlideRatio.CD0')
        self.connect('e', 'GlideRatio.e')
        self.connect('Takeoff.CLmax_TO_swept', 'GlideRatio.CLmax_TO_swept')
        # Missed approach
        self.add(submodel=MissedApproach(), name='MissedApproach', promotes=[])
        self.connect('Landing.CLmax_L_swept', 'MissedApproach.CLmax_L_swept')
        self.connect('Aeff', 'MissedApproach.Aeff')
        self.connect('nE', 'MissedApproach.nE')
        self.connect('CD0', 'MissedApproach.CD0')
        self.connect('e', 'MissedApproach.e')
        self.connect('mLbymTO', 'MissedApproach.mLbymTO')
        # Cruise matching
        self.add(submodel=CruiseMatching(), name='CruiseMatching', promotes=[])
        self.connect('Takeoff.mTObySW', 'CruiseMatching.mTObySW')
        self.connect('Aeff', 'CruiseMatching.Aeff')
        self.connect('VbyVmd', 'CruiseMatching.VbyVmd')
        self.connect('M', 'CruiseMatching.M')
        self.connect('BPR', 'CruiseMatching.BPR')
        self.connect('Emax', 'CruiseMatching.Emax')
        self.connect('CD0', 'CruiseMatching.CD0')
        self.connect('e', 'CruiseMatching.e')
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

        WbyS = csdl.max(WbyS_takeoff, WbyS_landing)
        self.register_output(name='WbyS', var=WbyS)
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
        self.register_output(name='CLmax_L_swept', var=CLmax_L_swept)

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
        self.parameters.declare(name='Lift-independent drag coefficient, slats', default=0.)
        self.parameters.declare(name='ke_gl', default=0.894,
                                desc='Factor for Oswald factor, glide')
        self.parameters.declare(name='Climb gradient', default=0.024)
        return

    def define(self):
        Aeff = self.declare_variable(name='Aeff', desc='Aspect ratio')
        nE = self.declare_variable(name='nE', desc='Number of engines')
        CD0 = self.declare_variable(name='CD0', desc='Lift-independent drag coefficient, clean')
        e_cr = self.declare_variable(name='e', desc='Oswald efficiency factor')

        # Lift coefficient at takeoff
        CLmax_TO_swept = self.declare_variable(name='CLmax_TO_swept',
                                               desc='Max lift coefficient at takeoff accounting for sweep')
        CLTO = CLmax_TO_swept / (1.2**2)

        # Drag coefficient increments
        dCD0_flap = 0.05*(CLTO-1.3)+0.01
        dCD0_slat = self.parameters['Lift-independent drag coefficient, slats']

        # Profile drag
        CDP = CD0 + dCD0_flap + dCD0_slat

        # Ostwald Efficiency factor, landing configuration
        e_l = e_cr * self.parameters['ke_gl']

        # Glide ratio in takeoff configuration
        L_D_TO = CLTO / (CDP+CLTO**2/(np.pi*Aeff*e_l))

        # Climb gradient
        sinGamma = self.parameters['Climb gradient']

        # Thrust-to-weight ratio
        TtobyWto = nE / (nE-1) * (1/L_D_TO+sinGamma)
        self.register_output(name='TtobyWto', var=TtobyWto)
        return


class MissedApproach(csdl.Model):

    def initialize(self):
        self.parameters.declare(name='dCD0_slat', default=0.,
                                desc='Lift-independent drag coefficient, slats')
        self.parameters.declare(name='dCD0_lg', default=0.015,
                                desc='Lift-independent drag coefficient, landing gear')
        self.parameters.declare(name='Climb gradient', default=0.024)
        self.parameters.declare(name='ke_ma', default=0.894,
                                desc='Factor for Oswald factor, missed approach')
        return

    def define(self):
        CLmax_L_swept = self.declare_variable(name='CLmax_L_swept')
        Aeff = self.declare_variable(name='Aeff', desc='Aspect ratio')
        nE = self.declare_variable(name='nE', desc='Number of engines')
        mLbymTO = self.declare_variable(name='mLbymTO', desc='Mass ratio, landing - take-off')
        CD0 = self.declare_variable(name='CD0', desc='Lift-independent drag coefficient, clean')
        e_cr = self.declare_variable(name='e', desc='Oswald efficiency factor')

        # Lift coefficient, landing
        CL_L = CLmax_L_swept / 1.3**2

        # Lift-independent drag coefficient, flaps
        dCD0_flap = 0.05*(CL_L-1.3)+0.01

        # Profile drag coefficient
        CD_P = CD0 + dCD0_flap + \
               self.parameters['dCD0_slat'] + self.parameters['dCD0_lg']

        # Oswald efficiency factor
        e_ma = self.parameters['ke_ma'] * e_cr

        # Glide ratio landing configuration
        E_L = CL_L/(CD_P+CL_L**2/np.pi/Aeff/e_ma)

        # Thrust-to-weight ratio
        TbyW_ma = nE / (nE - 1) * (1 / E_L + self.parameters['Climb gradient']) * mLbymTO
        self.register_output(name='TbyW_ma', var=TbyW_ma)
        return


class CruiseMatching(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='g', default=9.81, types=float)
        self.parameters.declare(name='gamma', default=1.4,
                                types=float, desc='Ratio of specific heats, air')
        self.parameters.declare(name='p0', default=101325., types=float,
                                desc='Air pressure, ISA, standard')
        return

    def define(self):
        mTObySW = self.declare_variable(name='mTObySW', desc='Wing loading at max takeoff mass')
        VbyVmd = self.declare_variable(name='VbyVmd')
        Aeff = self.declare_variable(name='Aeff', desc='Aspect ratio')
        M = self.declare_variable(name='M', desc='Mach number')
        BPR = self.declare_variable(name='BPR', desc='By-pass ratio')
        Emax = self.declare_variable(name='Emax', desc='Maximum glide ratio in cruise')
        CD0 = self.declare_variable(name='CD0', desc='Lift-independent drag coefficient, clean')
        e_cr = self.declare_variable(name='e', desc='Oswald efficiency factor')

        # Lift coefficient at Emax
        CLbyCLmd = 1 / VbyVmd ** 2
        CL_md = (CD0 * np.pi * Aeff * e_cr)**0.5

        # # Lift coefficient at cruise
        CL_cr = CLbyCLmd * CL_md
        self.register_output(name='CL_cruise', var=CL_cr)

        # Pressure at Cruise
        p_cr = mTObySW*2*self.parameters['g']/self.parameters['gamma']/CL_cr/M**2

        # Cruise altitude
        # h_cr = 11 + csdl.log10(p_cr/self.parameters['p0'] / 0.2232) * (-1 / 0.1577)
        h_cr = (1 - (p_cr / self.parameters['p0']) ** (1 / 5.256)) / 0.02256

        # Relative thrust in cruise
        TcrbyTto = (0.0013 * BPR - 0.0397) * h_cr - 0.0248 * BPR + 0.7125

        # L/D at cruise
        E = Emax * 2 / (1 / CLbyCLmd + CLbyCLmd)

        # Thrust-to-weight ratio at cruise
        TbyW_cr = 1/(TcrbyTto*E)
        self.register_output(name='TbyW_cr', var=TbyW_cr)
        return
# endregion


# region Mission Analysis
class MissionAnalysis(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='nm_to_m', default=1852., types=float)
        self.parameters.declare(name='g', default=9.81, types=float)

        self.parameters.declare(name='SFC cruise', default=1.65e-05,  # kg/N/s
                                types=float, desc='Spec.fuel consumption, cruise')  # todo: compute

        # region Regulation specified extra range
        self.parameters.declare(name='Loiter time', default=1800.,
                                types=float)  # 1800s for international, 2700s for domestic
        self.parameters.declare(name='s_alt', default=200.,
                                types=float, desc='Alternate flight distance')
        self.parameters.declare(name='s_longrange', default=5.,  # 5% for international, 0% for domestic
                                types=float, desc='Extra distance for long range in %')
        # endregion

        # region Emperical fuel fractions
        self.parameters.declare(name='Mff_taxi', default=0.997,
                                types=float, desc='Fuel-Fraction, taxi')
        self.parameters.declare(name='Mff_to', default=0.993,
                                types=float, desc='Fuel-Fraction, take-off')
        self.parameters.declare(name='Mff_clb', default=0.993,
                                types=float, desc='Fuel-Fraction, climb')
        self.parameters.declare(name='Mff_des', default=0.993,
                                types=float, desc='Fuel-Fraction, descent')
        self.parameters.declare(name='Mff_l', default=0.993,
                                types=float, desc='Fuel-Fraction, landing')
        # endregion

        # region Cargo and Passengers
        self.parameters.declare(name='m_pax', default=93.,
                                types=float, desc='Mass: one passengers, including baggage')
        self.parameters.declare(name='n_pax', default=180,
                                types=int, desc='Number of passengers')
        self.parameters.declare(name='m_cargo', default=2516.,
                                types=float, desc='Cargo mass')
        # endregion
        return

    def define(self):
        BPR = self.declare_variable(name='BPR', desc='By-pass ratio')
        VbyVmd = self.declare_variable(name='VbyVmd')
        Emax = self.declare_variable(name='Emax', desc='Maximum glide ratio in cruise')
        M = self.declare_variable(name='M', desc='Mach number')
        TbyW = self.declare_variable(name='TbyW', desc='Thrust loading')
        R = self.declare_variable(name='R', desc='Design range')  # nautical miles
        mLbymTO = self.declare_variable(name='mLbymTO', desc='Mass ratio, landing - take-off')
        CL_cr = self.declare_variable(name='CL_cr', desc='Cruise lift coefficient')
        Sw = self.declare_variable(name='Sw', desc='Wing area')

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
        rhocr = self.declare_variable(name='rhocr')
        self.connect('hcr', 'Atmosisa.z')
        self.connect('Atmosisa.speed_of_sound', 'acr')
        self.connect('Atmosisa.density', 'rhocr')
        # Tcr = 216.65
        # acr = 20.05 * Tcr**0.5
        Vcr = acr * M  # m/s
        self.register_output(name='Vcr', var=Vcr)

        # Cruise Lift
        Lcr = 0.5 * rhocr * Vcr ** 2 * CL_cr * Sw
        self.register_output(name='Cruise_lift', var=Lcr)

        # Mission ranges
        R = R * self.parameters['nm_to_m']  # m
        s_alt = self.parameters['s_alt'] * self.parameters['nm_to_m']  # m
        s_res = s_alt + self.parameters['s_longrange']/100 * R  # m

        sfc_cr = self.parameters['SFC cruise']  # kg/N/s

        # Cruise
        B_cruise = E * Vcr / sfc_cr / self.parameters['g']  # Breguet factor
        Mff_cr = csdl.exp(-R/B_cruise)  # Fuel fraction, cruise
        Mff_res = csdl.exp(-s_res/B_cruise)  # Fuel fraction, extra flight distance

        # Loiter
        t_loiter = self.parameters['Loiter time']  # s
        # sfc_loiter = sfc_cr
        B_loiter = B_cruise/Vcr
        Mff_loiter = csdl.exp(-t_loiter/B_loiter)

        #  Emperical fuel fractions
        Mff_taxi = self.parameters['Mff_taxi']
        Mff_to = self.parameters['Mff_to']
        Mff_clb = self.parameters['Mff_clb']
        Mff_des = self.parameters['Mff_des']
        Mff_l = self.parameters['Mff_l']

        # Final fuel fractions
        Mff_standard = Mff_to * Mff_clb * Mff_cr * Mff_des * Mff_l
        Mff_reserve = Mff_clb * Mff_res * Mff_loiter * Mff_des
        Mff_total = Mff_standard * Mff_reserve
        mFbymMTO = 1 - Mff_total  # Mission fuel fraction

        # Masses
        m_pax = self.parameters['m_pax']  # kg
        n_pax = self.parameters['n_pax']
        m_cargo = self.parameters['m_cargo']  # kg

        mOEbymTO = 0.23 + 1.04*TbyW  # Relative operating empty mass; 0.56 for A320-200
        m_pl = m_pax * n_pax + m_cargo
        m_mto = m_pl / (1-mFbymMTO-mOEbymTO)
        self.register_output(name='m_mto', var=m_mto)
        self.add_objective(name='m_mto', scaler=1e-4)

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
        constr_m_ml = m_ml_comp - m_ml
        self.register_output(name='Constraint_landing', var=constr_m_ml)
        self.add_constraint(name='Constraint_landing', upper=0., scaler=1e-2)

        # Constraint on cruise lift
        # Lift >= weight --> weight - lift <= 0
        Wcr = m_mto*(Mff_taxi*Mff_to*Mff_clb)*self.parameters['g']
        constr_cr_lift = Wcr - Lcr
        self.register_output(name='Constraint_cruise', var=constr_cr_lift)
        self.add_constraint(name='Constraint_cruise', upper=0., scaler=1e-5)
        return
# endregion


if __name__ == '__main__':
    sim = python_csdl_backend.Simulator(AircraftLevelSizing())
    # sim = csdl_om.Simulator(AircraftLevelSizing())

    # Aircraft design variables
    sim['Phi25'] = float(sys.argv[1])
    sim['Aeff'] = float(sys.argv[2])
    sim['lambdaeff'] = float(sys.argv[3])
    sim['dFo'] = float(sys.argv[4])
    sim['BPR'] = float(sys.argv[5])
    sim['tbyc'] = float(sys.argv[6])
    sim['ConstraintAnalysis.Landing.CLmax_L_unswept'] = float(sys.argv[7])
    sim['ConstraintAnalysis.Takeoff.CLmax_TO_unswept'] = float(sys.argv[8])
    # sim['MissionAnalysis.Emax'] = 17.48

    # Optimization Design Variables
    sim['VbyVmd'] = 0.94844796  # 0.94844796
    # sim['Sw_guess'] = 122.4

    # Operating conditions
    sim['M'] = float(sys.argv[9])
    sim['R'] = float(sys.argv[10])  # NM
    sim['nE'] = float(sys.argv[11])
    sim['ConstraintAnalysis.Landing.slfl'] = float(sys.argv[12])
    sim['ConstraintAnalysis.Landing.dTl'] = float(sys.argv[13])
    sim['ConstraintAnalysis.Takeoff.stofl'] = float(sys.argv[14])
    sim['ConstraintAnalysis.Takeoff.dTto'] = float(sys.argv[15])

    # Emperical values
    sim['mLbymTO'] = 0.878

    sim.run()
    # sim.visualize_implementation()
    print('------------------\n------------------')
    print('Approach speed (m/s): ', sim['ConstraintAnalysis.Landing.Vapp'])
    print('Wing loading at max. landing mass: ', sim['ConstraintAnalysis.Landing.mLbySW'])
    print('Wing loading at max. takeoff mass: ', sim['ConstraintAnalysis.Takeoff.mTObySW'])
    print('Thrust-to-weight ratio takeoff: ', sim['ConstraintAnalysis.Takeoff.TtobyWto'])
    print('Thrust-to-weight ratio glide: ', sim['ConstraintAnalysis.GlideRatio.TtobyWto'])
    print('Thrust-to-weight ratio missed approach: ', sim['ConstraintAnalysis.MissedApproach.TbyW_ma'])
    print('Thrust-to-weight ratio cruise matching: ', sim['ConstraintAnalysis.CruiseMatching.TbyW_cr'])
    print('------------------')
    print('Sizing thrust-to-weight ratio: ', sim['ConstraintAnalysis.TbyW'])
    print('Sizing wing loading: ', sim['ConstraintAnalysis.WbyS'])
    print('------------------')
    print('Maximum takeoff mass (kg) ', sim['m_mto'])
    print('Maximum landing mass (kg) ', sim['MissionAnalysis.m_ml'])
    print('Mission fuel mass (kg) ', sim['MissionAnalysis.m_f'])
    print('Total fuel mass (kg) ', sim['MissionAnalysis.m_f_erf'])
    print('Operating empty mass (kg) ', sim['MissionAnalysis.m_oe'])
    print('Wing area (m^2) ', sim['Sw'])
    print('Takeoff thrust, all engines (N) ', sim['Tto'])

    # Instantiate your problem using the csdl Simulator object and name your problem
    prob = CSDLProblem(problem_name='AircraftSizing', simulator=sim)
    # Setup your preferred optimizer (SLSQP) with the Problem object
    optimizer = SLSQP(prob, maxiter=100, ftol=1e-10)
    # Solve your optimization problem
    optimizer.solve()
    # Print results of optimization
    optimizer.print_results()
    
    sim['VbyVmd'] = optimizer.outputs['x'][-1, 0]
    sim.run()
    print('------------------\n------------------')
    print('Optimium DV value: ', optimizer.outputs['x'][-1, 0])
    print('Sizing thrust-to-weight ratio: ', sim['ConstraintAnalysis.TbyW'])
    print('Sizing wing loading: ', sim['ConstraintAnalysis.WbyS'])
    print('------------------')
    print('Maximum takeoff mass (kg) ', sim['m_mto'])
    print('Maximum landing mass (kg) ', sim['MissionAnalysis.m_ml'])
    print('Mission fuel mass (kg) ', sim['MissionAnalysis.m_f'])
    print('Total fuel mass (kg) ', sim['MissionAnalysis.m_f_erf'])
    print('Operating empty mass (kg) ', sim['MissionAnalysis.m_oe'])
    print('Wing area (m^2) ', sim['Sw'])
    print('Takeoff thrust, all engines (N) ', sim['Tto'])
    print('Landing constraint: ', sim['MissionAnalysis.Constraint_landing'])
    print('Cruise constraint: ', sim['MissionAnalysis.Constraint_cruise'])
    print('Fuel Tank Volume constraint: ', sim['FuelTankVolume.Constraint_FuelTankVolume'])
    # create some variables
    Landing_constraint = [sim['MissionAnalysis.Constraint_landing']]
    Cruise_constraint = [sim['MissionAnalysis.Constraint_cruise']]
    Fuel_Tank_Volume_constraint = [sim['FuelTankVolume.Constraint_FuelTankVolume']]
    Takeoff_thrust = [sim['Tto']]
    # save variables to a .mat file
    sio.savemat('variables.mat', {'a': Landing_constraint, 'b': Cruise_constraint, 'c':Fuel_Tank_Volume_constraint, 'd':Takeoff_thrust})


