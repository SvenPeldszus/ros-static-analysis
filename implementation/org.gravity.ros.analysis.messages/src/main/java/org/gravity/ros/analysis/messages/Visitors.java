package org.gravity.ros.analysis.messages;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.gravity.ros.analysis.messages.Dataclasses.AstModuleInfo;
import org.gravity.ros.analysis.messages.Dataclasses.RosAPIUsageInfo;
import org.gravity.ros.analysis.messages.Dataclasses.TopicInfo;
import org.python.pydev.parser.jython.Visitor;
import org.python.pydev.parser.jython.ast.Assign;
import org.python.pydev.parser.jython.ast.Attribute;
import org.python.pydev.parser.jython.ast.Call;
import org.python.pydev.parser.jython.ast.Str;
import org.python.pydev.parser.jython.ast.ClassDef;
import org.python.pydev.parser.jython.ast.Expr;
import org.python.pydev.parser.jython.ast.FunctionDef;
import org.python.pydev.parser.jython.ast.Name;
import org.python.pydev.parser.jython.ast.NameTok;
import org.python.pydev.parser.jython.ast.exprType;
import org.python.pydev.parser.jython.ast.stmtType;

public class Visitors {
	
	Dataclasses dataclasses = new Dataclasses();
	
	/*
	 * A visitor that pulls ROS API function from AST in FunctionDef format. 
	 */
	public class ROSApiVisitor extends Visitor {
		
		public FunctionDef api; // function we are looking for
		String funcName;
		String className = null;
		boolean inClass = false;
		
		/*
		 * To search for a function you need its name and which class it is in
		 */
		public ROSApiVisitor(String className, String funcName) {
			this.funcName = funcName;
			this.className = className;
			inClass = className == null;
		}
		
		@Override
		public Object visitClassDef(ClassDef node) throws Exception {
			Object result = null;
			if ( className != null && (((NameTok) node.name).id ).equals(className)) {
				inClass = true;
				result = super.visitClassDef(node);
				inClass = false;
			}
			return result;
		}
		
		@Override
		public Object visitFunctionDef(FunctionDef node) throws Exception {
			if (inClass && ((NameTok) node.name).id.equals(funcName))             
				api = node;
			
			// TODO: Is it possible to pass null here, so as not to go on parsing FuncDef ????
			return super.visitFunctionDef(node);
		}
	}
	
	
	/*
	 * Visitor that creates the {Topic - ROS Nodes} structure
	 * Explores AST of the project we are parsing, finding all ROS API function calls
	 */
	public class CallVisitor extends Visitor {
		List<FunctionDef> rosAPI;
		
		
		String moduleName = null; //name of the AST module that is in the visitor
		FunctionDef whereIsUsed; //function in which was created
		
		
		Map<String, TopicInfo> topicsMap = new HashMap<>(); //The structure we collect
		//Map<String, RosAPIUsageInfo> rosCalls = null; // { TOPIC - ... }
		
		Map<String, RosAPIUsageInfo> rosVars = null;
		
		public CallVisitor(List<FunctionDef> rosAPI) {
			this.rosAPI = rosAPI;
		}
		
		
		@Override
		public Object visitFunctionDef(FunctionDef node) throws Exception {
			whereIsUsed = node;
			rosVars = new HashMap<String, RosAPIUsageInfo>();
			//rosCalls = new HashSet<String>();
			
			Object result = super.visitFunctionDef(node);
			
			whereIsUsed = null;
			rosVars = null;
			//rosCalls = null;
			return result;
		}
		
		
		@Override
		public Object visitAssign(Assign node) throws Exception {
			
			/* Assign for only Publisher constructors, as Publisher constructor assign to variable */
			if (node.value instanceof Call) {
				Call call = (Call) node.value;
				String callFromName = ((Name) ((Attribute) call.func).value).id;
				
				//Publisher creation method must be called via the rospy library
				if (callFromName.equals("rospy")) {
					String callFuncName = ((NameTok) ((Attribute) call.func).attr).id;
					
					// If module name starts with UpperCase -> constructor
					if (Character.isUpperCase(callFuncName.charAt(0))) {
						
						for (FunctionDef funcDef: rosAPI) {
							//Constructor test conditions
							if (((NameTok) funcDef.name).id.equals("__init__") && 
									funcDef.parent instanceof ClassDef && 
									((NameTok)((ClassDef) funcDef.parent).name).id.equals(callFuncName)) {
								
								
								//1: Register publisher for concrete topic
								String topicName = ((Str) call.args[0]).s;
								
								TopicInfo topicInfo = topicsMap.get(topicName);
								if (topicInfo == null) {
									topicInfo = dataclasses.new TopicInfo();
									topicsMap.put(topicName, topicInfo);
								}
								if (topicInfo.publishers == null) {
									topicInfo.publishers = new LinkedList<RosAPIUsageInfo>();
								}
								RosAPIUsageInfo actualPublisher = dataclasses.new RosAPIUsageInfo(moduleName, whereIsUsed, node);
								topicInfo.publishers.add(actualPublisher);
								
								
								// 2: Register the name of the variable where we save ROS node, 
								// as it accesses the rospy module
								for (exprType target: node.targets) {
									if (rosVars != null)
										rosVars.put(((Name) target).id, actualPublisher);
										//rosVars.add(();
									
									
									actualPublisher.rosVar = ((Name) target).id;
									break;
								}
								
								
								// 1: Assemble Assign to Function Def
//								Collection<stmtType> callCollection = callsApiMap.get(funcDef);
//								if (callCollection == null) {
//									callCollection = new LinkedList<stmtType>();
//									callsApiMap.put(funcDef, callCollection);
//								}
//								callCollection.add(node);
							}
						}
					}
				}
			}
			
			
			return super.visitAssign(node);
		}
		
		
		@Override
		public Object visitExpr(Expr node) throws Exception {
			if (node.value instanceof Call) {
				Call call = (Call) node.value;
				if (call.func instanceof Attribute) {
					String callFromName = ((Name) ((Attribute) call.func).value).id;
					
					if (callFromName.equals("rospy") || rosVars.get(callFromName) != null) {
						String callFuncName = ((NameTok) ((Attribute) call.func).attr).id;
						
						// Case for Subscriber generators
						if (Character.isUpperCase(callFuncName.charAt(0))) {
							
							for (FunctionDef funcDef: rosAPI) {
								// Constructor test conditions
								if (((NameTok) funcDef.name).id.equals("__init__") &&
										funcDef.parent instanceof ClassDef && 
										((NameTok)((ClassDef) funcDef.parent).name).id.equals(callFuncName)) {
									
									
									//1: Register subscriber for concrete topic
									String topicName = ((Str) call.args[0]).s;
									
									TopicInfo topicInfo = topicsMap.get(topicName);
									if (topicInfo == null) {
										topicInfo = dataclasses.new TopicInfo();
										topicsMap.put(topicName, topicInfo);
									}
									if (topicInfo.subscribers == null) {
										topicInfo.subscribers = new LinkedList<RosAPIUsageInfo>();
									}
									RosAPIUsageInfo actualSubscriber = dataclasses.new RosAPIUsageInfo(moduleName, whereIsUsed, node);
									topicInfo.subscribers.add(actualSubscriber);
									
									
									// Assemble Call to Function Def
//									Collection<stmtType> callCollection = callsApiMap.get(funcDef);
//									if (callCollection == null) {
//										callCollection = new LinkedList<stmtType>();
//										callsApiMap.put(funcDef, callCollection);
//									}
//									callCollection.add(node);
								}
							}
						}
						
						// Case for publish method
						else {
							for (FunctionDef funcDef: rosAPI) {
								
								// Publish method conditions
								if (((NameTok) funcDef.name).id.equals(callFuncName)) {
									
									if (rosVars.get(callFromName) != null)
									{
										if (rosVars.get(callFromName).usageFunction == null) {
											rosVars.get(callFromName).usageFunction = new LinkedList<stmtType>();
										}
										rosVars.get(callFromName).usageFunction.add(node);
									}
									
									
//									Collection<stmtType> callCollection = callsApiMap.get(funcDef);
//									if (callCollection == null) {
//										callCollection = new LinkedList<stmtType>();
//										callsApiMap.put(funcDef, callCollection);
//									}
//									callCollection.add(node);
								}
							}
						}
					}
				}
			}
			
			return super.visitExpr(node);
		}
	}
}
