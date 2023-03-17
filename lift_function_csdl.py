import csdl
import python_csdl_backend


class LiftFunction(csdl.Model):
    def initialize(self):
        self.parameters.declare(name='V', default=400.)
        self.parameters.declare(name='e', default=3.)
        return

    def define(self):

        V = self.parameters['V']
        e = self.parameters['e']

        Cl = self.declare_variable(name='Cl', val=0.01)
        S = self.declare_variable(name='S', val=100)

        L = 0.5 * e * V**2 * Cl * S

        self.register_output(name='L', var=L)
        return


if __name__ == '__main__':
    sim = python_csdl_backend.Simulator(LiftFunction())
    sim.run()
    print(sim['L'])
    rep = csdl.GraphRepresentation(LiftFunction())
    rep.visualize_graph()
