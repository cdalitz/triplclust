//
// graph.cpp
//     Classes and functions for computing the MST and for
//     splitting up clusters at gaps
//
// Author:  Jens Wilberg, Lukas Aymans, Christoph Dalitz
// Date:    2018-08-30
// License: see ../LICENSE
//

#include <time.h>
#include <algorithm>
#include <stack>

#include "graph.h"

struct Edge {
  size_t src, dest;
  double weight;
  bool operator<(const Edge &e2) const { return this->weight < e2.weight; };
};

// Create edges with weights between all point indices in *cluster*. the
// weights are the distances of the points in *cloud*.  The edges are returned
// in *edges*.
void create_edges(std::vector<Edge> &edges, const PointCloud &cloud,
                  const std::vector<size_t> &cluster) {
  for (size_t vertex1 = 0; vertex1 < cluster.size(); ++vertex1) {
    for (size_t vertex2 = vertex1 + 1; vertex2 < cluster.size(); ++vertex2) {
      size_t point_index1 = cluster[vertex1], point_index2 = cluster[vertex2];
      const Point &p = cloud[point_index1];
      const Point &q = cloud[point_index2];

      // compute squared distance
      double distance = (q - p).squared_norm();

      Edge e = {vertex1, vertex2, distance};
      edges.push_back(e);
    }
  }
  std::sort(edges.begin(), edges.end());
}

// Create a adjacent list for every vertex from the edges in *edges*.
// The adjacent list is returned in *adj* which be resized to the number of
// vertices a priori.
void create_adj(std::vector<std::vector<size_t> > &adj,
                const std::vector<Edge> edges) {
  for (std::vector<Edge>::const_iterator edge = edges.begin();
       edge != edges.end(); ++edge) {
    adj[edge->src].push_back(edge->dest);
    adj[edge->dest].push_back(edge->src);
  }
}

// Remove all edges of *edges* with weight smaller *dmax*. *edges* will be
// modified.
void remove_edge(std::vector<Edge> &edges, double dmax) {
  std::vector<Edge>::iterator edge = edges.begin();
  double dmax2 = dmax * dmax;
  while (edge != edges.end()) {
    if (edge->weight > dmax2) {
      edge = edges.erase(edge);
    } else {
      ++edge;
    }
  }
}

// Remove all edges from *edges* which don't belong to the mst. *vcount* is the
// number of verteicies. *edges* will be modified.
void mst(const std::vector<Edge> &edges, std::vector<Edge> &mst_edges,
         size_t vcount) {
  std::vector<size_t> groups(vcount);
  for (size_t i = 0; i < groups.size(); i++) {
    groups[i] = i;
  }

  for (std::vector<Edge>::const_iterator e = edges.begin(); e != edges.end();
       ++e) {
    size_t group_a = groups[e->src];
    size_t group_b = groups[e->dest];

    // look if there is no circle
    if (group_a != group_b) {
      // merge groups
      for (std::vector<size_t>::iterator it = groups.begin();
           it != groups.end(); ++it) {
        if (*it == group_b) *it = group_a;
      }
      mst_edges.push_back(*e);
    }
  }
}

// create a cluster of connected components which is returned in *new_cluster*.
// *vertex* is the index of the start vertex. *visited* is a list of the visted
// states from every vertecy. *cluster* is used to get the original point index
// of a vertex. *adj* are the adjacent lists of all vertices.
void dfs_util(std::vector<size_t> &new_cluster, const size_t vertex,
              std::vector<bool> &visited, const std::vector<size_t> &cluster,
              const std::vector<std::vector<size_t> > &adj) {
  std::stack<size_t> stack;
  stack.push(vertex);
  while (!stack.empty()) {
    size_t v = stack.top();
    stack.pop();
    visited[v] = true;
    new_cluster.push_back(cluster[v]);

    for (std::vector<size_t>::const_iterator it = adj[v].begin();
         it != adj[v].end(); ++it) {
      if (!visited[*it]) stack.push(*it);
    }
  }
}

//-------------------------------------------------------------------
// Split *cluster* in multiple new clusters and return the result in
// *new_clusters". The mst of the cluster is created and all edges are
// removed with a wheigth > *dmax*. The connected components are computed
// and returned as new clusters if their size is >= *min_size*.
//-------------------------------------------------------------------
void max_step(std::vector<std::vector<size_t> > &new_clusters,
              const std::vector<size_t> &cluster, const PointCloud &cloud,
              double dmax, size_t min_size) {
  size_t vcount = cluster.size();
  size_t n_removed;
  std::vector<std::vector<size_t> > adj(vcount);
  std::vector<Edge> edges, mst_edges;
  std::vector<bool> visited(vcount);
  create_edges(edges, cloud, cluster);
  mst(edges, mst_edges, vcount);
  n_removed = mst_edges.size();
  remove_edge(mst_edges, dmax);
  n_removed = n_removed - mst_edges.size();
  create_adj(adj, mst_edges);

  for (size_t v = 0; v < vcount; ++v) {
    if (!visited[v]) {
      std::vector<size_t> new_cluster;
      dfs_util(new_cluster, v, visited, cluster, adj);
      if ((new_cluster.size() >= min_size) || (n_removed == 0))
        new_clusters.push_back(new_cluster);
    }
  }
}
