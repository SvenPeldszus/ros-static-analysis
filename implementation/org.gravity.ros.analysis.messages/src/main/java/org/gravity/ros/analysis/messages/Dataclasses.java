package org.gravity.ros.analysis.messages;

import java.util.Collection;
import java.util.List;

import org.python.pydev.parser.jython.ast.Call;
import org.python.pydev.parser.jython.ast.FunctionDef;
import org.python.pydev.parser.jython.ast.stmtType;
import org.python.pydev.shared_core.parsing.BaseParser.ParseOutput;

public class Dataclasses {
	
	/* 
	 * Dataclass that saves the module name for ast each file 
	 */
	public class AstModuleInfo {
		public ParseOutput ast;
		public String moduleName;
		
		public AstModuleInfo(ParseOutput ast, String moduleName) {
			this.ast = ast;
			this.moduleName = moduleName;
		}
	};
	
	/* 
	 * Dataclass that stores usage information of ROS Node (for publisher or subscriber)
	 */
	public class RosAPIUsageInfo {
		public String moduleName; //name of the module in which the ROS Node was created
		public FunctionDef whereIsUsed; //function in which was created
		public stmtType rosAPIConstructor; //Call of the constructor itself
		
		//Functions that belong to the ROS Node. For publisher this is the publish method
		public List<stmtType> usageFunction = null;
		public String rosVar = null; // Only for publisher (Name of object)

		public RosAPIUsageInfo(String moduleName, FunctionDef whereIsUsed, stmtType rosAPIConstructor) {
			this.moduleName = moduleName;
			this.whereIsUsed = whereIsUsed;
			this.rosAPIConstructor = rosAPIConstructor;
		}
	};
	
	public class TopicInfo {
		public Collection<RosAPIUsageInfo> publishers;
		public Collection<RosAPIUsageInfo> subscribers;
	}; 
}
