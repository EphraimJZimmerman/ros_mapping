import networkx as nx
import matplotlib.pyplot as plt

# Create a directed graph object
G = nx.DiGraph()

# Add nodes
G.add_node(1)
G.add_node(2)
G.add_node(3)
G.add_node(4)


# Add directed edges (from -> to)
G.add_edge(1, 2)
G.add_edge(2, 3)
G.add_edge(3, 4)
G.add_edge(4, 1)

# Optionally, add weights to the edges
G[1][2]['weight'] = 4
G[2][3]['weight'] = 5
G[3][4]['weight'] = 6
G[4][1]['weight'] = 7

# Visualize the graph
nx.draw(G, with_labels=True, node_color='lightblue', font_weight='bold', node_size=2000, arrows=True)
plt.show()

# Print the graph nodes and edges
print("Nodes:", G.nodes())
print("Edges:", G.edges())

# If you want to print edge data (e.g., weights):
for u, v, data in G.edges(data=True):
    print(f"Edge from {u} to {v} with weight {data['weight']}")
