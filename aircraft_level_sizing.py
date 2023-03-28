import numpy as np

import csdl
import python_csdl_backend
import csdl_om


class AircraftLevelSizing(csdl.Model):
    def initialize(self):
        return
    def define(self):
        Phi25 = self.declare_variable(name='Phi25', units='deg', desc='Sweep angle, at 25% of chord')
        self.register_output(name='Phi25_out', var=Phi25*1.)

        # Landing
        self.add(submodel=Landing(), name='Landing', promotes=[])
        self.connect('Phi25_out', 'Landing.Phi25')
        # Takeoff
        self.add(submodel=Takeoff(), name='Takeoff', promotes=[])
        self.connect('Phi25_out', 'Takeoff.Phi25')
        self.connect('Landing.mLbySW', 'Takeoff.mLbySW')
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

        # Wing loading at max takeoff mass
        mTObySW = mLbySW/mLbymTO
        self.register_output(name='mTObySW', var=mTObySW)

        # Slope
        a = self.parameters['kto'] / (stofl * s * CLmax_TO_swept)

        # Thrust-to-weight ratio
        TtobyWto = mTObySW * a
        self.register_output(name='TtobyWto', var=TtobyWto)
        return


if __name__ == '__main__':
    sim = python_csdl_backend.Simulator(AircraftLevelSizing())

    sim['Phi25'] = 25.

    sim['Landing.slfl'] = 1448.
    sim['Landing.dTl'] = 0.
    sim['Landing.CLmax_L_unswept'] = 3.76

    sim['Takeoff.mLbymTO'] = 0.878
    sim['Takeoff.stofl'] = 1768.
    sim['Takeoff.dTto'] = 0.
    sim['Takeoff.CLmax_TO_unswept'] = 2.85

    sim.run()
    print('Approach speed (m/s): ', sim['Landing.Vapp'])
    print('Wing loading at max. landing mass: ', sim['Landing.mLbySW'])
    print('Wing loading at max. takeoff mass: ', sim['Takeoff.mTObySW'])
    print('Thrust-to-weight ratio: ', sim['Takeoff.TtobyWto'])

    rep = csdl.GraphRepresentation(AircraftLevelSizing())
    rep.visualize_graph()




