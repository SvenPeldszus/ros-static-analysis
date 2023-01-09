package org.gravity.ros.analysis.messages;

import java.util.Collection;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

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
		public String folderName;
		
		public Map<String, FunctionInfo> moduleFunctions = new HashMap<String, FunctionInfo>(); // Functions deklared in module
		Map<String, String> aliases = new HashMap<String, String>(); // All aliases deklared in module
		
		public AstModuleInfo(ParseOutput ast, String moduleName, String folderName) {
			this.ast = ast;
			this.moduleName = moduleName;
			this.folderName = folderName;
		}
	};
	
	
	/*
	 * A dataclass that stores the properties of a function.
	 */
	public class FunctionInfo {
		public String moduleName;
		public String folderName;
		public String functionName;
		public Map<Integer, List<Call>> publishCalls = new HashMap<Integer, List<Call>>();
		
		public FunctionInfo(String moduleName, String folderName, String functionName) {
			this.moduleName = moduleName;
			this.folderName = folderName;
			this.functionName = functionName;
		}
	};

	
	/*
	 * Dataclass that stores usage information of Publisher ROS Node
	 */
	public class PublisherInfo {
		public FunctionInfo whereIsUsed; //function in which was created
		public String varName = null;  //name of the object to which the generator has been assigned
		public List<Call> usageFunction = new LinkedList<Call>(); //Calls to all publish() methods
		
		public PublisherInfo(FunctionInfo whereIsUsed) {
			this.whereIsUsed = whereIsUsed;
		}		
	};
	
	
	/*
	 * Dataclass that stores usage information of Subscriber ROS Node
	 */
	public class SubscriberInfo {
		public FunctionInfo whereIsUsed; //function in which was created
		public FunctionInfo callbackFunction; //function for receiving messages from Publisher
		
		public SubscriberInfo(FunctionInfo whereIsUsed) {
			this.whereIsUsed = whereIsUsed;
		}
	};
	
	
	/*
	 * A collection of all publisher and subscriber for a particular topic. 
	 * The topic name is stored in another structure as HashMap<String, TopicInfo>.
	 */
	public class TopicInfo {
		public Collection<PublisherInfo> publishers;
		public Collection<SubscriberInfo> subscribers;
	}; 
}
