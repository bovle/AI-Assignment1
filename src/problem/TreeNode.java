package problem;

import java.util.ArrayList;
import java.util.List;

public class TreeNode {
    public RobotConfig data;
    public TreeNode parent;
    public List<TreeNode> children;
    public List<RobotConfig> pathToParent;

    public TreeNode(RobotConfig data, TreeNode parent, List<RobotConfig> pathToParent) {
        this.data = data;
        this.parent = parent;
        this.children = new ArrayList<>();
        this.pathToParent = pathToParent;
        if (parent != null) {
            parent.children.add(this);
        }
    }

}