import networkx as nx
import itertools
import numpy as np
from matplotlib import pyplot as plt
import matplotlib as mpl
import pandas as pd
from pyrkm.circuit_utils import Circuit


class RKMNetwork:
    def __init__(self, numV = 2, numH = 1):
        self._NV = numV
        self._NH = numH
        self.generate(self._NV, self._NH)
        self.generate_full(self._NV, self._NH)
        self.set_keys()

        
#         self._NE = len(self._graph.edges())
#         self._NN = len(self._graph.nodes())
       
    @property
    def NV(self):
        return self._NV
    
    @NV.setter
    def NV(self, value):
        self._NV = value
        self.generate(self._NV, self._NH, doset=True)
        self.generate_full(self._NV, self._NH, doset=True)
        self.set_keys()
    
    @property
    def NH(self):
        return self._NH
    
    @NH.setter
    def NH(self, value):
        self._NH = value
        self.generate(self._NV, self._NH, doset=True)
        self.generate_full(self._NV, self._NH, doset=True)
        self.set_keys()
        
    @property
    def edge(self):
        return self._edge
        
#     @edge.setter
#     def edge(self, value):
#         self._edge = value
        
    @property
    def node(self):
        return self._node
        
#     @node.setter
#     def node(self, value):
#         self._node = value
        
    @property
    def graph(self):
        return self._graph
        
#     @graph.setter
#     def graph(self, value):
#         self._graph = value
        
    @property
    def pos(self):
        return self._pos
        
#     @pos.setter
#     def pos(self, value):
#         self._pos = value  
        
    @property
    def fullgraph(self):
        return self._fullgraph
        
#     @graph.setter
#     def fullgraph(self, value):
#         self._fullgraph = value
# #         self._NE = len(self._graph.edges())
#         self._NN = len(self._graph.nodes())
        
    @property
    def fullpos(self):
        return self._fullpos
        
#     @pos.setter
#     def fullpos(self, value):
#         self._fullpos = value  
        
#     @property
#     def NE(self):
#         return self._NE
    
#     @property
#     def NN(self):
#         return self._NN

    def draw_network(self, full=False, **kwargs):
        if full:
            G = self._fullgraph
            pos = self._fullpos
        else:
            G = self._graph
            pos = self._pos
        print(G)
        fig, ax = plt.subplots()
        nx.draw_networkx(G, pos=pos, node_size=800, ax=ax, **kwargs)
        return fig, ax
    
    
    def draw_network_state(self, edgeweights=None, nodeweights=None, erange = None, nrange = None,
                           e_cmap='bwr', n_cmap='RdYlBu', **kwargs):

        if edgeweights is None:
            edgeweights = self.get_ks()
        
        ecm = plt.get_cmap(e_cmap)
        ncm = plt.get_cmap(n_cmap)
        
        if erange is None:
            erange = [-128, 128]
            #[np.min(edgeweights), np.max(edgeweights)]
            
        if nrange is None:
            nrange = [np.min(nodeweights), np.max(nodeweights)]

        fig, ax = plt.subplots()
        nx.draw_networkx(self._graph, pos=self._pos, node_size=800, ax=ax,
                                   width= edgeweights/np.max(edgeweights)*10, edge_cmap = ecm, edge_color = edgeweights, 
                                    node_color = nodeweights, cmap = ncm)
        
#         fig.set_figwidth(12)
        
        
        norm = mpl.colors.Normalize(vmin=erange[0],vmax=erange[1])
        sm = plt.cm.ScalarMappable(cmap=ecm, norm=norm)
        sm.set_array([])
        fig.colorbar(sm, ticks=np.linspace(erange[0],erange[1],7), ax=ax, location='right', label='Edge k')
        
#         norm2 = mpl.colors.Normalize(vmin=nrange[0],vmax=nrange[1])
#         sm2 = plt.cm.ScalarMappable(cmap=ncm, norm=norm2)
#         sm.set_array([])
#         fig.colorbar(sm2, ticks=np.linspace(nrange[0],nrange[1],7), ax=ax, location='left', label='Nodes [V]')
        
        ax.collections[0].set_edgecolor('k') 
        
        return fig, ax

    def draw_fullnetwork_state(self, edgeweights=None, nodeweights=None, erange = None, nrange = None,
                           e_cmap='jet', n_cmap='RdYlBu', **kwargs):

        if edgeweights is None:
            edgeweights= self.get_conductances()
        
        ecm = plt.get_cmap(e_cmap)
        ncm = plt.get_cmap(n_cmap)
        
        if erange is None:
            erange = [1e-5, .00128]
#             [np.min(edgeweights), np.max(edgeweights)]
            
        if nrange is None:
            nrange = [np.min(nodeweights), np.max(nodeweights)]

        fig, ax = plt.subplots()
        nx.draw_networkx(self._fullgraph, pos=self._fullpos, node_size=800, ax=ax,
                                   #width= edgeweights/np.max(edgeweights)*10, 
                                 width=2.,
                                 edge_cmap = ecm, edge_color = edgeweights, 
                                    node_color = nodeweights, cmap = ncm)
        
#         fig.set_figwidth(12)
        
        
        norm = mpl.colors.Normalize(vmin=erange[0],vmax=erange[1])
        sm = plt.cm.ScalarMappable(cmap=ecm, norm=norm)
        sm.set_array([])
        fig.colorbar(sm, ticks=np.linspace(erange[0],erange[1],7), ax=ax, location='right', label=r'Edge conductances [1/$\Omega$]')
        
#         norm2 = mpl.colors.Normalize(vmin=nrange[0],vmax=nrange[1])
#         sm2 = plt.cm.ScalarMappable(cmap=ncm, norm=norm2)
#         sm.set_array([])
#         fig.colorbar(sm2, ticks=np.linspace(nrange[0],nrange[1],7), ax=ax, location='left', label='Nodes [V]')
        
        ax.collections[0].set_edgecolor('k') 
        
        return fig, ax
    
    def generate_full(self, lenV=2, lenH=1, ysep = 2., lrxsep = 2., midxsep = 4., pmsep = 0.4, doset=True):
        
        G = nx.Graph()
        
        nodes = ['L+', 'L-', 'R+', 'R-']

        edges = []
        for vi in range(lenV):
            nodes.append('V{}+'.format(vi))
            nodes.append('V{}-'.format(vi))

            edges.append(('L+', 'V{}+'.format(vi)))
            edges.append(('L-', 'V{}+'.format(vi)))

        for hi in range(lenH):
            nodes.append('H{}+'.format(hi))
            nodes.append('H{}-'.format(hi))

            edges.append(('R+', 'H{}+'.format(hi)))
            edges.append(('R-', 'H{}+'.format(hi)))

        for n in nodes:
            G.add_node(n)

        for vi, hi in itertools.product(np.arange(lenV), np.arange(lenH)):
#             print(vi, hi)
            edges.append(('V{}+'.format(vi), 'H{}+'.format(hi)))
            edges.append(('V{}+'.format(vi), 'H{}-'.format(hi)))
            edges.append(('V{}-'.format(vi), 'H{}+'.format(hi)))

        G.add_edges_from(edges)


        pos = {'L-': [0, -pmsep*2], 'L+': [0,pmsep*2], 
               'R-': [lrxsep*2 + midxsep, -pmsep*2], 'R+':[lrxsep*2+midxsep, pmsep*2]}
             
        midV = lenV/2.
        midH = lenH/2.

        if lenV%2==0:
            Vystart = .5 - midV
        else:
            Vystart = -midV+0.5

        if lenH%2==0:
            Hystart = .5 - midH
        else:
            Hystart = -midH+0.5

        for vi in range(lenV):
            pos['V{}+'.format(vi)] = [lrxsep, -(vi+Vystart)*ysep+pmsep]
            pos['V{}-'.format(vi)] = [lrxsep, -(vi+Vystart)*ysep-pmsep]

        for hi in range(lenH):
            pos['H{}+'.format(hi)] = [lrxsep + midxsep, -(hi+Hystart)*ysep+pmsep]
            pos['H{}-'.format(hi)] = [lrxsep+ midxsep, -(hi+Hystart)*ysep-pmsep]

        attrs = {}
        for key, val in pos.items():
            name = key[:-1]
            if key[-1] == '+':
                pm = 1
            else:
                pm = 0
            attrs[key] = {'pos': val, 'name': name, 'pm': pm}

        nx.set_node_attributes(G, attrs)

        attrs = {}

        for edge in G.edges:
            a,b = edge
            if 'L' in a or 'R' in a:
                if a[1] == '+':
                    pm = 1
                else:
                    pm = 0
                name = 'B' + b[:2]
            else:
                if '-' in a or '-' in b:
                    pm = 0
                else:
                    pm = 1
                name = 'W' + a[1] + b[1]

        #     print(a,b)
        #     print(name)

            attrs[edge] = {'name': name, 'pm': pm}

        nx.set_edge_attributes(G, attrs)
            
        if doset:
            self._fullgraph = G
            self._fullpos = pos
            
        return G, pos
    
    def generate(self, lenV=2, lenH=1, ysep = 2., lrxsep = 2., midxsep = 4., pmsep = 0.4, doset=True):
        
        G = nx.Graph()
        
        nodes = ['L', 'R']

        edges = []
        for vi in range(lenV):
            nodes.append('V{}'.format(vi))

            edges.append(('L', 'V{}'.format(vi)))

        for hi in range(lenH):
            nodes.append('H{}'.format(hi))

            edges.append(('R', 'H{}'.format(hi)))

        for n in nodes:
            G.add_node(n)

        for vi, hi in itertools.product(np.arange(lenV), np.arange(lenH)):
#             print(vi, hi)
            edges.append(('V{}'.format(vi), 'H{}'.format(hi)))

        G.add_edges_from(edges)


        pos = {'L': [0, 0],  'R': [lrxsep*2 + midxsep, 0]}
             
        midV = lenV/2.
        midH = lenH/2.

        if lenV%2==0:
            Vystart = .5 - midV
        else:
            Vystart = -midV+0.5

        if lenH%2==0:
            Hystart = .5 - midH
        else:
            Hystart = -midH+0.5

        for vi in range(lenV):
            pos['V{}'.format(vi)] = [lrxsep, -(vi+Vystart)*ysep]

        for hi in range(lenH):
            pos['H{}'.format(hi)] = [lrxsep + midxsep, -(hi+Hystart)*ysep]
            
        attrs = {}

        for edge in G.edges:
            a,b = edge
            if 'L' in a or 'R' in a:
                name = 'B' + b[:2]
            else:
                name = 'W' + a[1] + b[1]

#             print(a,b)
#             print(name)

        #     k = np.sign(np.random.uniform(-1., 1.))
            k = np.random.randint(-128, 128)
#             print(k)

            attrs[edge] = {'name': name, 'k': k}

        nx.set_edge_attributes(G, attrs)

            
        if doset:
            self._graph = G
            self._pos = pos
            
        return G, pos
    
    def set_k(self, edgename, val):
        edge = [(u,v) for u,v,e in self._graph.edges(data=True) if e['name'] == edgename][0]
        
        nx.set_edge_attributes(self._graph, {edge: {"k": val}})
#         print(self._graph.edges[edge])

    def load_state(self, row):
        for key in row.keys():
            if key[0] == 'B' or key[0] == 'W':
#                 print(key)
                val = row[key]
#                 print(val)
                self.set_k(key, val)

    def get_ks(self):
        ks = []
        for u,v,e in self._graph.edges(data=True):
            k = e['k']
            ks.append(k)
        return ks
        
    def get_conductances(self):
        conductances = []
        for e in self._fullgraph.edges:
            name = self._fullgraph.edges[e]['name']
            pm = self._fullgraph.edges[e]['pm']


            k = [e['k'] for u,v,e in self._graph.edges(data=True) if e['name'] == name][0]

            if pm != int(k>0):
                realres = 1e5 #ohms
            else:
                realval = np.abs(k)
                realres = 1e5*(1-realval/128.)

            realk = 1./realres

            conductances.append(realk)
        return conductances
    
    def set_keys(self):
        edgekeys = []
        isweight = []
        Anodekeys = []
        Bnodekeys = []

        for edge in self.graph.edges:
            a, b = edge
            if (a=='L') or (a=='R'):
                edgekeys.append('B'+b)
                isweight.append(0)
                Anodekeys.append('A0')
                Bnodekeys.append(b)
            else:
                vi = a.split('V')[1]
                hi = b.split('H')[1]
                edgekeys.append('W'+vi+hi)
                isweight.append(1)
                Anodekeys.append(a)
                Bnodekeys.append(b)

        nodekeys = np.unique([Anodekeys, Bnodekeys])

#         print(edgekeys)
#         print(isweight)
#         print(Anodekeys)
#         print(Bnodekeys)
#         print(nodekeys)
        
        ekeydf = pd.DataFrame({'name': edgekeys, 'isWeight': isweight, 'Anode': Anodekeys, 'Bnode': Bnodekeys})
        nkeydf = pd.DataFrame({'name':nodekeys})
#                  , 'node':nodekeys}
#         print(keydf)
        
        self._edge = ekeydf
        self._node = nkeydf

    def clamp(self, FB=0, vals = [1,1], analogRails = [1.0, 3.0]):
        aZero = np.mean(analogRails)
        aPlus = analogRails[1]
        aMinus = analogRails[0]
        
        # boundary nodes
#         indices = np.where(['L' in x or 'R' in x for x in rkm.fullgraph.nodes])[0]
        indices = [0,1,2,3]
        clampvals = [aPlus, aMinus, aPlus, aMinus]
#         print(indices, clampvals)
        
        for nodename in self._node.name:
            if ('V' in nodename and FB==0) or ('H' in nodename and FB==1):
                idx = int(nodename[1:])
                val = vals[idx]
                fullnodes = [u for u,n in self._fullgraph.nodes(data=True) if n['name'] == nodename]
                for fn in fullnodes:
                    nodeidx = [i for i, x in enumerate(self._fullgraph.nodes) if x==fn][0]
#                     print(fn, nodeidx)
                    pm = self._fullgraph.nodes[fn]['pm']
#                     print(pm)
                    indices.append(nodeidx)
                    if val == 0:
                        cval = aZero
                    else:
                        if pm:
                            cval = aPlus
                        else:
                            cval = aMinus
                    clampvals.append(cval)
        return np.array(indices), np.array(clampvals)
    
    def clamp_solve(self, FB=0, vals = [1,1], analogRails = [1.0, 3.0], plot=True):
        c = Circuit(self._fullgraph)

        conductances = self.get_conductances()

        c.setConductances(conductances)
        
        indices_nodes, f = self.clamp(FB=FB, vals = vals, analogRails = analogRails)
        Q = c.constraint_matrix(indices_nodes)
        V = c.solve(Q, f)
        
        if plot:
            self.draw_fullnetwork_state(conductances, nodeweights=-1*V)
        return dict(zip(self._fullgraph.nodes, V))
    
    def run_nodechecks(self, nA, nB):
        check = True
        nA = np.array(nA)
        nB = np.array(nB)
        for node in self.node.name:
            Ainst = [i for i, val in enumerate(self.edge.Anode) if val == node]
            Binst = [i for i, val in enumerate(self.edge.Bnode) if val == node]

            ninst = np.concatenate([nA[Ainst], nB[Binst]])

            if not(np.all(ninst == ninst[0])):
                check = False

        return check