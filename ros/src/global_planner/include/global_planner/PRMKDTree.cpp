/*
*	File: PRMKDTree.cpp
*	---------------
*   PRM KDTree Implementation
*/

#include <global_planner/PRMKDTree.h>
#include "PRMKDTree.h"
namespace PRM{
	KDTree::KDTree(){
		this->size_ = 0;
	}

	void KDTree::clear(){
		this->root_.reset();
		this->size_ = 0;
		this->notTarget_.clear();
		this->notTargetTemp_.clear();
		this->notTargetPerm_.clear();
	}

	std::shared_ptr<Node> KDTree::getRoot(){
		return this->root_;
	}

	int KDTree::getSize(){
		return this->size_;
	}

	void KDTree::insert(std::shared_ptr<Node> n) {
		// set child pointers to NULL (optional if Node constructor does this)
		n->left = nullptr;
		n->right = nullptr;

		// if tree is empty, add node as root
		if (this->size_ == 0) {
			this->root_ = n;
			this->size_++;
			return;
		}

		std::shared_ptr<Node> ptr = this->root_;
		int depth = 0;

		while (true) {
			int index = depth % 3; // Use dimension_ if supporting more than 3D
			double value = ptr->pos(index);
			double insertValue = n->pos(index);

			if (insertValue > value) {
				if (ptr->right == nullptr) {
					ptr->right = n;
					n->treeParent = ptr;
					this->size_++;
					return;
				}
				ptr = ptr->right;
			} else if (insertValue < value) {
				if (ptr->left == nullptr) {
					ptr->left = n;
					n->treeParent = ptr;
					this->size_++;
					return;
				}
				ptr = ptr->left;
			} else { // Avoid infinite loop
				return;
			}

			depth++;
		}
	}

	// 辅助函数：递归实现最近邻搜索
	std::shared_ptr<Node> KDTree::nearestNeighborHelper(
		std::shared_ptr<Node> query,
		std::shared_ptr<Node> current,
		int depth,
		double &bestDist,
		std::shared_ptr<Node> bestNode,
		const std::unordered_set<std::shared_ptr<Node>> &exclude)
	{
		if (!current) return bestNode;

		// 计算当前节点与查询点的距离（排除自身或已排除节点）
		int axis = depth % 3; // 若支持任意维度，可使用成员变量 dimension_
		double d = (query->pos - current->pos).norm();
		if (current == query || exclude.count(current))
			d = std::numeric_limits<double>::infinity();

		// 更新最优候选
		if (d < bestDist) {
			bestDist = d;
			bestNode = current;
		}

		// 根据当前轴决定“好侧”（near）和“坏侧”（far）
		std::shared_ptr<Node> nearBranch = nullptr, farBranch = nullptr;
		if (query->pos[axis] < current->pos[axis]) {
			nearBranch = current->left;
			farBranch  = current->right;
		} else {
			nearBranch = current->right;
			farBranch  = current->left;
		}

		// 先递归搜索好侧
		bestNode = nearestNeighborHelper(query, nearBranch, depth + 1, bestDist, bestNode, exclude);

		// 如果超平面距离小于当前最小距离，则可能有更近的点，搜索坏侧
		if (farBranch && std::abs(query->pos[axis] - current->pos[axis]) < bestDist) {
			bestNode = nearestNeighborHelper(query, farBranch, depth + 1, bestDist, bestNode, exclude);
		}
		return bestNode;
	}

	// 修改后的 nearestNeighbor：构造排除集合，并调用辅助函数
	std::shared_ptr<Node> KDTree::nearestNeighbor(
		std::shared_ptr<Node> query,
		std::shared_ptr<Node> rootNode, 
		std::shared_ptr<Node> bestNode,
		int depth)
	{
		if (!rootNode)
			rootNode = this->root_;
		if (!rootNode)
			return bestNode; // 空树

		// 构造排除集合：将成员级的 notTargetTemp_ 与 notTargetPerm_ 合并
		std::unordered_set<std::shared_ptr<Node>> exclude = this->notTargetPerm_;
		exclude.insert(this->notTargetTemp_.begin(), this->notTargetTemp_.end());

		double bestDist = std::numeric_limits<double>::infinity();
		return nearestNeighborHelper(query, rootNode, depth, bestDist, bestNode, exclude);
	}

	// 修改后的 kNearestNeighbor：每次调用 nearestNeighbor 时将已找到的节点排除
	std::vector<std::shared_ptr<Node>> KDTree::kNearestNeighbor(std::shared_ptr<Node> query, int num)
	{
		if (!query)
			throw std::invalid_argument("Input node cannot be null");

		std::vector<std::shared_ptr<Node>> knn;
		// 不能返回查询点本身，所以最多返回 size_ - 1 个邻居
		num = std::min(this->size_ - 1, num);

		// 局部保存已排除的节点，避免影响全局状态
		std::unordered_set<std::shared_ptr<Node>> localExclude = this->notTargetPerm_;
		// 清空临时排除集合（确保每次从空状态开始）
		this->notTargetTemp_.clear();

		for (int i = 0; i < num; ++i) {
			// 更新成员级临时排除集合，以便 nearestNeighbor 在内部构造排除集合时能排除已经找到的邻居
			this->notTargetTemp_ = localExclude;

			auto nearest = nearestNeighbor(query, nullptr, nullptr, 0);
			if (!nearest) {
				std::cout << "\033[1;31m[KD-Tree]:" << " Size = " << this->getSize() << "\033[0m" << std::endl;
				std::vector<int> heights = this->checkBalance();
				std::cout << "\033[1;31m[KD-Tree]:" << " Tree Height = " << heights[0]  << " Left Height = " << heights[1] << " Right Height = " << heights[2] << "\033[0m" << std::endl;
				std::cout << "\033[1:31m[KD-Tree]:" << " notTargetTemp_ size = " << this->notTargetTemp_.size() << "\033[0m" << std::endl;
				std::cout << "\033[1:31m[KD-Tree]:" << " notTargetPerm_ size = " << this->notTargetPerm_.size() << "\033[0m" << std::endl;
				std::cout << "\033[1:31m[KD-Tree]:" << " Root node: " << this->root_->pos.transpose() << "\033[0m" << std::endl;
				if (query != nullptr) {
					std::cout << "\033[1:31m[KD-Tree]:" << " Query node: " << query->pos.transpose() << "\033[0m" << std::endl;
				} else {
					std::cout << "\033[1:31m[KD-Tree]:" << " Query node is null" << "\033[0m" << std::endl;
				}

				this->notTargetTemp_.clear();
				throw std::runtime_error("Failed to find neighbor at iteration " + std::to_string(i));
			}
			knn.push_back(nearest);
			localExclude.insert(nearest);
		}
		this->notTargetTemp_.clear();
		return knn;
	}

	bool KDTree::isDegenerate() const {
		std::queue<std::shared_ptr<Node>> q;
		if (root_) q.push(root_);
		
		while (!q.empty()) {
			auto node = q.front();
			q.pop();
			
			// 发现分支点立即返回
			if (node->left && node->right) return false;
			
			if (node->left) q.push(node->left);
			if (node->right) q.push(node->right);
		}
		return true; // 全树无分支
	}
	
	void KDTree::remove(std::shared_ptr<Node> n){
			this->notTargetPerm_.insert(n);
			--this->size_;
	}

	std::vector<int> KDTree::checkBalance() const {
		if (isDegenerate()) {
			throw std::runtime_error("KDTree degenerated to linked list");
		}
	
		// Height balance check
		std::function<int(std::shared_ptr<Node>)> height = [&](std::shared_ptr<Node> node) {
			if (!node) return 0;
			return 1 + std::max(height(node->left), height(node->right));
		};
	
		return std::vector<int>{height(root_), height(root_->left), height(root_->right)};
	}

	void KDTree::printTree(std::shared_ptr<Node> node, int indent=0) {
		if (!node) return;
		std::cout << std::string(indent, ' ') 
				<< "Node(" << node->pos.transpose() << ")" << std::endl;
		printTree(node->left, indent + 2);
		printTree(node->right, indent + 2);
	}
}
