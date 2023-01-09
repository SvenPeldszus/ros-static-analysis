package org.gravity.ros.analysis.messages;

import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import org.eclipse.jdt.internal.compiler.classfmt.ModuleInfo;
import org.gravity.ros.analysis.messages.Dataclasses.AstModuleInfo;
import org.gravity.ros.analysis.messages.Dataclasses.FunctionInfo;
import org.gravity.ros.analysis.messages.Dataclasses.PublisherInfo;
import org.gravity.ros.analysis.messages.Dataclasses.SubscriberInfo;
import org.gravity.ros.analysis.messages.Dataclasses.TopicInfo;
import org.python.pydev.parser.jython.Visitor;
import org.python.pydev.parser.jython.ast.Assign;
import org.python.pydev.parser.jython.ast.Attribute;
import org.python.pydev.parser.jython.ast.Call;
import org.python.pydev.parser.jython.ast.Str;
import org.python.pydev.parser.jython.ast.ClassDef;
import org.python.pydev.parser.jython.ast.Expr;
import org.python.pydev.parser.jython.ast.FunctionDef;
import org.python.pydev.parser.jython.ast.ImportFrom;
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
		
		public FunctionDef api; //The function that the Visitor saves
		String apiFuncName;
		String apiClassName = null;
		boolean inClass; //= false; TODO:
		
		/*
		 * To search for a function you need its name and which class it is in
		 */
		public ROSApiVisitor(String className, String funcName) {
			this.apiFuncName = funcName;
			this.apiClassName = className;
			inClass = className == null;
		}
		
		/*
		 * If the class name is the same as the api function class name, we go inside the class and look for FunctionDef
		 */
		@Override
		public Object visitClassDef(ClassDef node) throws Exception {
			// TODO: inClass exit
			
			Object result = null;
			String className = ((NameTok) node.name).id;
			
			if ( apiClassName != null && className.equals(apiClassName)) {
				inClass = true;
				result = super.visitClassDef(node);
				inClass = false;
			}
			
			return result;
		}
		
		/*
		 * If the inClass flag is set, compare the function with the api function
		 */
		@Override
		public Object visitFunctionDef(FunctionDef node) throws Exception {
			String funcName = ((NameTok) node.name).id;
			
			if (inClass && funcName.equals(apiFuncName))             
				api = node;
			
			return null; // TODO:
			//return super.visitFunctionDef(node);
		}
	}
	
	
	/*
	 * Visitor that creates the {Topic - ROS Nodes} structure
	 * Explores AST of the project we are parsing, finding all ROS API function calls
	 */
	public class CallVisitor extends Visitor {
		
		Map<String, TopicInfo> topicsMap = new HashMap<>(); //The structure we collect
		
		List<FunctionDef> rosAPI;
		public AstModuleInfo moduleInfo;
		public Map<String, FunctionInfo> globalFunctions; //Functions of all modules 
		
		FunctionInfo whereIsUsed; //The function in which the Ros Node constructor was called
		Map<String, PublisherInfo> rosPublishers = null;
		
		
		public CallVisitor(List<FunctionDef> rosAPI) {
			this.rosAPI = rosAPI;
		}
		
		
		/*
		 * By entering the function we make a new HashMap for Publishers, 
		 * which can be created in the function
		 */
		@Override
		public Object visitFunctionDef(FunctionDef node) throws Exception {
			String funcName = ((NameTok) node.name).id;
			
			whereIsUsed = moduleInfo.moduleFunctions.get(funcName);
			rosPublishers = new HashMap<String, PublisherInfo>();
			
			Object result = super.visitFunctionDef(node);
			
			whereIsUsed = null;
			rosPublishers = null;
			
			return result;
		}
		
		
		/*
		 * Assign for only Publisher constructors, as Publisher constructor assign to variable
		 */
		@Override
		public Object visitAssign(Assign node) throws Exception {
			
			if (node.value instanceof Call) {
				Call call = (Call) node.value;
				String callNamespace = ((Name) ((Attribute) call.func).value).id;
				
				//Publisher creation method must be called via the rospy library
				if (callNamespace.equals("rospy")) {
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
									topicInfo.publishers = new LinkedList<PublisherInfo>();
								}
								PublisherInfo actualPublisher = dataclasses.new PublisherInfo(whereIsUsed);
								topicInfo.publishers.add(actualPublisher);
								
								
								// 2: Register the name of the variable where we save ROS node, 
								// as it accesses the rospy module
								for (exprType target: node.targets) {
									if (rosPublishers != null)
										rosPublishers.put(((Name) target).id, actualPublisher);
									
									actualPublisher.varName = ((Name) target).id;
									break;
								}
							}
						}
					}
				}
			}
			return super.visitAssign(node);
		}
		
		
		/*
		 * VisitExpr covers cases for calls to the Subscriber constructor as well as the publish() method
		 */
		@Override
		public Object visitExpr(Expr node) throws Exception {
			
			if (node.value instanceof Call) {
				Call call = (Call) node.value;
				
				if (call.func instanceof Attribute) {
					String callFromName = ((Name) ((Attribute) call.func).value).id;
					
					if (callFromName.equals("rospy") || rosPublishers.get(callFromName) != null) {
						String callFuncName = ((NameTok) ((Attribute) call.func).attr).id;
						
						/*** Case for Subscriber generators ***/
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
										topicInfo.subscribers = new LinkedList<SubscriberInfo>();
									}
									SubscriberInfo actualSubscriber = dataclasses.new SubscriberInfo(whereIsUsed);
									topicInfo.subscribers.add(actualSubscriber);
									
									
									//2: Find right callback
									//function was called from class
									if (call.args[2] instanceof Attribute) {
										Attribute a = (Attribute) call.args[2]; 
										String callbackName = resolveAlias(((NameTok) a.attr).id);
										String callbackNamespace = resolveAlias(((Name) a.value).id);
										
										actualSubscriber.callbackFunction = resolveFunction(callbackNamespace + "." + callbackName);
									}
									//function was called by itself
									else if (call.args[2] instanceof Name) {
										Name n = (Name) call.args[2]; 
										String callbackName = resolveAlias(n.id);
										
										actualSubscriber.callbackFunction = resolveFunction(callbackName);
									}
									
								}
							}
						}
						
						/*** Case for publish() method ***/
						else {
							for (FunctionDef funcDef: rosAPI) {
								
								// Publish method conditions
								if (((NameTok) funcDef.name).id.equals(callFuncName)) {
									
									if (rosPublishers.get(callFromName) != null)
									{
										if (rosPublishers.get(callFromName).usageFunction == null) {
											rosPublishers.get(callFromName).usageFunction = new LinkedList<Call>();
										}
										rosPublishers.get(callFromName).usageFunction.add(call);
									}
								}
							}
						}
					}
				}
			}
			
			return super.visitExpr(node);
		}
		
		/*
		 * We Check functions to see if publisher has been passed. 
		 * If so, see if the publish() method was called inside the function
		 */
		@Override
		public Object visitCall(Call node) throws Exception {
			int i = 0;
			for (exprType arg: node.args) {
				
				if (arg instanceof Name && node.func instanceof Name) {
					String argName = ((Name) arg).id;
					PublisherInfo publisher = rosPublishers.get(argName);
					
					if (publisher != null) {
						String funcName = ((Name) node.func).id;
						FunctionInfo func = resolveFunction(resolveAlias(funcName));
						
						if (func != null) {
							List<Call> listCall = func.publishCalls.get(i);
							if (listCall != null) {
								publisher.usageFunction.addAll(listCall);
							}
						}
						
					}
				}
				i++;
			}
			
			return super.visitCall(node);
		}
		
		public String resolveAlias(String symbolName) {
			String result = moduleInfo.aliases.get(symbolName);
			if (result != null && !result.equals(symbolName))
				return resolveAlias(result);
			return symbolName;
		}

		public FunctionInfo resolveFunction(String callbackName) {
			FunctionInfo result = moduleInfo.moduleFunctions.get(callbackName);
			if (result == null) 
				result = globalFunctions.get(callbackName);
			return result;
			
		}
	}
	
	
	/*
	 * A vizitor that bypasses all functions and saves their value. 
	 *   - Location by module and folder, as well as the name of the function. 
	 *   - In Map globalFunctions and moduleFunctions saves also the namepace of the class in which it is located.
	 *   - Additionally, it saves a sturcture for function parameters for the publish method call. 
	 */
	public class FunctionVisitor extends Visitor {
		
		public Map<String, FunctionInfo> globalFunctions = new HashMap<String, FunctionInfo>();
		public FunctionInfo functionInfo;
		public AstModuleInfo moduleInfo;
		String namespace;
		Map<String, Integer> args = null;
		
		/*
		 * Save class namespace
		 */
		@Override
		public Object visitClassDef(ClassDef node) throws Exception {
			Object result = null;
			
			namespace = ((NameTok) node.name).id;
			result = super.visitClassDef(node);
			
			namespace = null;
		
			return result;
		}
		
		/*
		 * Save the function separately to moduleFunctions for the special module, 
		 * and also save it to the globalFunctions structure.
		 * Write down all function parameters by name and number, so that we can find calls to publish() method later
		 */
		@Override
		public Object visitFunctionDef(FunctionDef node) throws Exception {
			String functionName = ((NameTok) node.name).id;
			
			functionInfo = dataclasses. new FunctionInfo(moduleInfo.moduleName, moduleInfo.folderName, functionName);
			
			//Add class namespace if needed
			if (namespace != null)
				functionName = namespace + "." + functionName;
			
			globalFunctions.put(functionName, functionInfo);
			moduleInfo.moduleFunctions.put(functionName, functionInfo);
			
			//Get a list of parameters
			args = new HashMap<String, Integer>();
			int i = 0;
			for (exprType arg: node.args.args) {
				args.put(((Name) arg).id, i);
				i++;
			}
			
			super.visitFunctionDef(node);
			
			args = null;
			functionInfo = null;
			
			return null;
		}
		
		
		/*
		 * Check if there is a call to the publish() method. 
		 * If there is - save as - number of function parameter: list<call>
		 */
		@Override
		public Object visitExpr(Expr node) throws Exception {
			if (node.value instanceof Call) {
				Call call = (Call) node.value;
				
				if (call.func instanceof Attribute && ((Attribute) call.func).attr instanceof NameTok) {
					
					//Checking whether a call is a call to the publish method
					if (((NameTok)((Attribute) call.func).attr).id.equals("publish")) {
						
						String callFromName = ((Name)((Attribute) call.func).value).id;
						
						//Check if the map arguments have this value
						Integer arg = args.get(callFromName);
						if (arg != null) {
							List<Call> callList = functionInfo.publishCalls.get(arg);
							if (callList == null) {
								callList = new LinkedList<Call>();
								functionInfo.publishCalls.put(arg, callList);
							}
							callList.add(call);
						}
					}
				}
				
			}
			return super.visitExpr(node);
		}
	}
	
	
	public class AliasingVisitor extends Visitor {
		public AstModuleInfo moduleInfo;
		
		@Override
		public Object visitClassDef(ClassDef node) throws Exception {
			return null;
		}
		
		@Override
		public Object visitFunctionDef(FunctionDef node) throws Exception {
			return null;
		}
		
		@Override
		public Object visitImportFrom(ImportFrom node) throws Exception {
			if (node.names[0].asname != null) {
				String asName = ((NameTok) node.names[0].name).id;
				String originalName = ((NameTok) node.names[0].asname).id;
				moduleInfo.aliases.put(originalName, asName);
			}
			
			return super.visitImportFrom(node);
		}
		
		@Override
		public Object visitAssign(Assign node) throws Exception {
			if (node.value instanceof Name) {
				String originalName = ((Name) node.targets[0]).id;
				String asName = ((Name) node.value).id;
				moduleInfo.aliases.put(originalName, asName);
			}
			return super.visitAssign(node);
		}
	}
}
