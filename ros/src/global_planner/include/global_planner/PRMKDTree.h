/*
*	File: PRMKDTree.h
*	---------------
*   PRM KDTree header
*/

#ifndef PRMKDTREE_H
#define PRMKDTREE_H
#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <queue>


using std::cout; using std::endl;
namespace PRM{
	struct Node{
		Eigen::Vector3d pos;
		int numVoxels = 0;
		bool isNavTarget = false;
		double pathCost = std::numeric_limits<double>::infinity();
		std::unordered_map<double, int> yawNumVoxels = {};
		std::shared_ptr<Node> left = NULL;
		std::shared_ptr<Node> right = NULL;
		// std::weak_ptr<Node> treeParent; 
		std::shared_ptr<Node> treeParent = NULL;
		std::shared_ptr<Node> parent = NULL;
		double g = std::numeric_limits<double>::infinity();
		double f = std::numeric_limits<double>::infinity();
		
		bool newNode = false;
		std::unordered_set<std::shared_ptr<Node>> adjNodes;

		Node (const Eigen::Vector3d& p){
			this->pos = p;
		}

		int getUnknownVoxels(double angle){
			if (int(yawNumVoxels.size()) == 0){
				return 0;
			}
			// make sure the angle is [0, 2PI)
			while (angle < 0){
				angle += 2 * M_PI;
			}

			while (angle >= 2 * M_PI){
				angle -= 2 * M_PI;
			}

			double startYaw, endYaw;
			std::vector<double> yaws;
			for (auto key : yawNumVoxels){
				yaws.push_back(key.first);
			}
			std::sort(yaws.begin(), yaws.end());

			double yawDiff = yaws[1] - yaws[0];
			bool findAngle = false;
			for (int i=0; i<int(yaws.size())-1; ++i){
				if (angle >= yaws[i] and angle <= yaws[i+1]){
					startYaw = yaws[i];
					endYaw = yaws[i+1];
					findAngle = true;
					break;
				}
			}

			// this means it is in [last angle, zero] range
			if (not findAngle){
				startYaw = yaws.back();
				endYaw = 0.0;
			}

			int angleNumVoxels = yawNumVoxels[startYaw] + (angle - startYaw) * double(yawNumVoxels[endYaw] - yawNumVoxels[startYaw])/yawDiff;
			return angleNumVoxels;
		}

		double getBestYaw(){
			int maxNum = 0;
			double maxYaw = 0;
			for (auto key : yawNumVoxels){
				if (key.second > maxNum){
					maxNum = key.second;
					maxYaw = key.first;
				}
			}
			return maxYaw;
		}

		int getBestYawVoxel(){
			if (int(yawNumVoxels.size()) == 0){
				return 0;
			}

			double bestYaw = this->getBestYaw();
			return this->yawNumVoxels[bestYaw];
		}
	};

	struct CompareNode{
		bool operator()(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2){
			return n1->f > n2->f;
		}
	};

	struct GainCompareNode{
		bool operator()(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2){
			return n1->numVoxels < n2->numVoxels;
		}
	};


	class KDTree{
	private:
		int size_;
		std::shared_ptr<Node> root_;
		std::vector<std::shared_ptr<Node>> notTarget_;
		// std::unordered_set<std::shared_ptr<Node>> notTargetTemp_;
		// std::unordered_set<std::shared_ptr<Node>> notTargetPerm_;

	public:
		KDTree();


		std::unordered_set<std::shared_ptr<Node>> notTargetTemp_;
		std::unordered_set<std::shared_ptr<Node>> notTargetPerm_;


		void clear();
		std::shared_ptr<Node> getRoot();
		int getSize();
		void insert(std::shared_ptr<Node> n);
        std::shared_ptr<Node> nearestNeighborHelper(std::shared_ptr<Node> query, std::shared_ptr<Node> current, int depth, double &bestDist, std::shared_ptr<Node> bestNode, const std::unordered_set<std::shared_ptr<Node>> &exclude);
        std::shared_ptr<Node> nearestNeighbor(std::shared_ptr<Node> n,
                                              std::shared_ptr<Node> rootNode = NULL,
                                              std::shared_ptr<Node> bestNode = NULL,
                                              int depth = 0);
        std::vector<std::shared_ptr<Node>> kNearestNeighbor(std::shared_ptr<Node> n, int num);
        bool isDegenerate() const;
        void remove(std::shared_ptr<Node> n);
        // int checkBalance() const;
		std::vector<int> checkBalance() const;

        void printTree(std::shared_ptr<Node> node, int indent);
    };
}

#endif