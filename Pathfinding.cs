using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;

public class Pathfinding : MonoBehaviour {

	Grid grid;

	void Awake() {
		grid = GetComponent<Grid>();
	}

	public void FindPath(PathRequest request, Action<PathResult> callback) {

		Stopwatch sw = new Stopwatch();
		sw.Start();

		Vector3[] waypoints = new Vector3[0];
		bool pathSuccess = false;

		Node startNode = grid.NodeFromWorldPoint(request.pathStart);
		Node targetNode = grid.NodeFromWorldPoint(request.pathEnd);

		if (startNode.walkable && targetNode.walkable) {
			Heap<Node> openSet = new Heap<Node>(grid.MaxSize);
			HashSet<Node> closedSet = new HashSet<Node>();
			openSet.Add(startNode);

			while (openSet.Count > 0) {
				Node node = openSet.RemoveFirst();
				closedSet.Add(node);

				if (node == targetNode) {
					sw.Stop();
					print("Path found in " + sw.ElapsedMilliseconds + " ms");
					pathSuccess = true;
					break;
				}

				foreach (Node neighbor in grid.GetNeighbors(node)) {
					if (!neighbor.walkable || closedSet.Contains(neighbor)) {
						continue;
					}

					int newCostToneighbor = node.gCost + GetDistance(node, neighbor) + neighbor.movementPenalty;
					if (newCostToneighbor < neighbor.gCost || !openSet.Contains(neighbor)) {
						neighbor.gCost = newCostToneighbor;
						neighbor.hCost = GetDistance(neighbor, targetNode);
						neighbor.parent = node;

						if (!openSet.Contains(neighbor))
							openSet.Add(neighbor);
						else
							openSet.UpdateItem(neighbor);
					}
				}
			}
		}
		if (pathSuccess) {
			waypoints = RetracePath(startNode, targetNode);
			pathSuccess = waypoints.Length > 0;
		}
		callback(new PathResult(waypoints, pathSuccess, request.callback));
	}

	Vector3[] RetracePath(Node startNode, Node endNode) {
		List<Node> path = new List<Node>();
		Node currentNode = endNode;

		while (currentNode != startNode) {
			path.Add(currentNode);
			currentNode = currentNode.parent;
		}
		Vector3[] waypoints = SimplifyPath(path);
		Array.Reverse(waypoints);
		return waypoints;
	}

	Vector3[] SimplifyPath(List<Node> path) {
		List<Vector3> waypoints = new List<Vector3>();
		Vector2 directionOld = Vector2.zero;

		for (int i = 1; i < path.Count; i ++) {
			Vector2 directionNew = new Vector2(path[i - 1].gridX - path[i].gridX, path[i - 1].gridY - path[i].gridY);
			if (directionNew != directionOld) {
				waypoints.Add(path[i].worldPosition);
			}
			directionOld = directionNew;
		}
		return waypoints.ToArray();
	}

	int GetDistance(Node nodeA, Node nodeB) {
		int dstX = Mathf.Abs(nodeA.gridX - nodeB.gridX);
		int dstY = Mathf.Abs(nodeA.gridY - nodeB.gridY);

		if (dstX > dstY)
			return 14*dstY + 10*(dstX - dstY);
		return 14*dstX + 10*(dstY - dstX);
	}
}