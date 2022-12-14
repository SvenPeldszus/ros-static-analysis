package org.gravity.ros.analysis.messages;

import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.json.*;
import org.eclipse.core.internal.resources.File;
import org.python.pydev.parser.jython.ast.Call;
import org.python.pydev.parser.jython.ast.Assign;
import org.python.pydev.parser.jython.ast.exprType;
import org.python.pydev.parser.jython.ast.stmtType;
import org.python.pydev.parser.jython.ast.Attribute;
import org.python.pydev.parser.jython.ast.Name;
import org.eclipse.core.resources.IFile;
import org.eclipse.core.resources.IFolder;
import org.eclipse.core.resources.IProject;
import org.eclipse.core.resources.IResource;
import org.eclipse.core.resources.IResourceVisitor;
import org.eclipse.core.resources.IWorkspace;
import org.eclipse.core.resources.IWorkspaceRoot;
import org.eclipse.core.resources.ResourcesPlugin;
import org.eclipse.core.runtime.CoreException;
import org.eclipse.core.runtime.IPath;
import org.eclipse.core.runtime.IProgressMonitor;
import org.eclipse.core.runtime.Path;
import org.eclipse.jface.text.IDocument;
import org.eclipse.ui.editors.text.TextFileDocumentProvider;
import org.eclipse.ui.texteditor.IDocumentProvider;
import org.python.pydev.core.IPythonNature;
import org.python.pydev.core.IPythonPathNature;
import org.python.pydev.core.MisconfigurationException;
import org.python.pydev.parser.PyParser;
import org.python.pydev.parser.PyParser.ParserInfo;
import org.python.pydev.parser.jython.SimpleNode;
import org.python.pydev.parser.jython.Visitor;
import org.python.pydev.parser.jython.ast.Call;
import org.python.pydev.parser.jython.ast.ClassDef;
import org.python.pydev.parser.jython.ast.Expr;
import org.python.pydev.parser.jython.ast.FunctionDef;
import org.python.pydev.parser.jython.ast.Module;
import org.python.pydev.parser.jython.ast.NameTok;
import org.python.pydev.parser.jython.ast.VisitorBase;import org.python.pydev.parser.jython.ast.VisitorIF;
import org.python.pydev.plugin.nature.PythonNature;
import org.python.pydev.shared_core.model.ISimpleNode;
import org.python.pydev.shared_core.parsing.BaseParser.ParseOutput;

public class PythonProjectParser {

	public List<ParseOutput> parse(IProject project) throws CoreException{
		
		// Get the PythonNature for the IProject we are going to parse. After collect all files and folders of project
		IPythonPathNature iPythonPathNature = PythonNature.getPythonPathNature(project);
		IPythonNature iPythonNature = iPythonPathNature.getNature();
		Set<IResource> iResourceList = (Set<IResource>) iPythonPathNature.getProjectSourcePathFolderSet();

		// IDocumentProvider is needed to translate IResource to IDocument. 
		// We need IDocument as an input to the PyParser.createCythonAst() function
		IDocumentProvider provider = new TextFileDocumentProvider();
		
		// Collection of all ast's of files in project we need to return
		LinkedList<ParseOutput> parsedList = new LinkedList<>();
		
		
		for (IResource scr : iResourceList) {
			scr.accept(new IResourceVisitor() {
				
				@Override
				public boolean visit(IResource resource) throws CoreException {
					// We do not go into the libs folder, because that is where the linked ROS modules, etc. are located
					// Folder libs is not part of the project.
					if (resource instanceof IFolder && resource.getName().equals("libs")) 
						return false;
					
					// Go through the files and select python modules ignoring __init__ files
					if (resource instanceof IFile) {
						IFile ifile = (IFile) resource;
						if (ifile.getFileExtension().equals("py") && !ifile.getName().equals("__init__.py")) {
							System.out.println(ifile.getName()); // File name output for debugging
							IDocument document = createIDocument(ifile); // We need an IDocument for the parser

							// Parsing the needed file with PyParser
							try {
								ParseOutput parseOutput = PyParser.reparseDocument(new ParserInfo(document, iPythonNature));
								parsedList.add(parseOutput);
							} catch (MisconfigurationException e) { e.printStackTrace(); }
						}
					}
					return true;
				}
			});
		}
		return parsedList;
	}
	
	
	class ROSApiVisitor extends Visitor {
		public FunctionDef api; 
		String funcName;
		String className = null;
		boolean inClass = false;
		
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
	
	
	public List<FunctionDef> getRosAPI(IProject project) throws Exception {
		
		// TODO: Make json file retrieval by relative path
		// Collect all the key parameters of ROS functions from the Json file.
		String content = Files.readString(Paths.get("D:\\Coding\\JavaEclipse\\ros-static-analysis\\implementation\\org.gravity.ros.analysis.messages\\src\\main\\java\\org\\gravity\\ros\\analysis\\messages\\ROSApi.json"));
		JSONArray jsonArray = new JSONArray(content);
		
		List<FunctionDef> api = new LinkedList<>();

		// Bypass each ROS function described in the json
		for (Object o : jsonArray) {
			JSONObject funcDef = (JSONObject) o;
		    
		    // Create the path to the desired file
		    String sitePackagesPath = getSitePackagesPath();
		    String filename = (String) funcDef.get("fileName");
		    String fullPath = sitePackagesPath + "/" + filename;
		    
		    // Create a link to the IFile of the desired module 
		    java.io.File file = new java.io.File(fullPath);
			IFolder iFolder = project.getFolder("libs");
			if (!iFolder.exists())
				iFolder.create(true, true, null);
			IFile iFile = iFolder.getFile(filename);
			if (!iFile.exists()) 
				createLink(file, iFile, null);
			
			// Create an AST for the desired file
			IDocument document = createIDocument(iFile);
			Module ast = (Module) PyParser.reparseDocument(new ParserInfo(document, PythonNature.getPythonNature(project))).ast;
			
			// Declare the necessary input values for the visitor class
		    String funcName = (String) funcDef.get("funcName");
		    String className;
		    if (funcDef.get("className").equals(null))
		    	className = null;
		    else 
		    	className = (String) funcDef.get("className");
		    
		    ROSApiVisitor visitor = new ROSApiVisitor(className, funcName);
		    ast.accept(visitor);
		    
		    api.add(visitor.api);
		}
		
		return api;
	}
	
	/*
	 *  FuncDef (__init__ (publisher)) : {pub = rospy.Publisher, pub2 = rospy.Publisher, 3 node}
	 *        + pub.publish
	 *  __init__ (subscriber)
	 *  
	 *  
	 *  IN: MAP
	 *  for publisher suche publish -> pub publish
	 *                                   topicname {chatter}
	 *  for subscriber suche callback -> callback
	 *  
	 *  
	 *  { Function Defenition - Publisher: {1 - pub = rospy.Publisher, 2 - pub2 = rospy.Publisher};
	 *    Function Defenition - publish: {1 - 7 - pub.publish, 8 - pub2.publish}
	 *  }
	 *  
	 *  
	 *  
	 *  
	 *  
	 *  
	 *  
	 *  
	 *  
	 */
	
	class CallVisitor extends Visitor {
		List<FunctionDef> rosAPI;
		Map<FunctionDef, Collection<stmtType>> callsApiMap;
		Set<String> rosVars = null;
		
		public CallVisitor(List<FunctionDef> rosAPI) {
			this.rosAPI = rosAPI;
			this.callsApiMap = new HashMap<>();
		}
		
		@Override
		public Object visitFunctionDef(FunctionDef node) throws Exception {
			rosVars = new HashSet<String>(); 
			Object result = super.visitFunctionDef(node);
			rosVars = null;
			return result;
		}
		
		@Override
		public Object visitAssign(Assign node) throws Exception {
			/* Assign for only Publisher constructors, as Publisher constructor assign to variable */
			if (node.value instanceof Call) {
				Call call = (Call) node.value;
				String callModuleName = ((Name) ((Attribute) call.func).value).id;
				
				if (callModuleName.equals("rospy")) {
					String callFuncName = ((NameTok) ((Attribute) call.func).attr).id;
					
					// If module name starts with UpperCase -> constructor
					if (Character.isUpperCase(callFuncName.charAt(0))) {
						for (FunctionDef funcDef: rosAPI) {
							if (((NameTok) funcDef.name).id.equals("__init__") && 
									funcDef.parent instanceof ClassDef && 
									((NameTok)((ClassDef) funcDef.parent).name).id.equals(callFuncName)) {
								// 1: Assemble Assign to Function Def
								Collection<stmtType> callCollection = callsApiMap.get(funcDef);
								if (callCollection == null) {
									callCollection = new LinkedList<stmtType>();
									callsApiMap.put(funcDef, callCollection);
								}
								callCollection.add(node);
								
								// 2: Register the name of the variable where we save ROS node, 
								// as it accesses the rospy module
								for (exprType target: node.targets) {
									if (rosVars != null)
										rosVars.add(((Name) target).id);   
									break;
								}
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
					String callModuleName = ((Name) ((Attribute) call.func).value).id;
					
					if (callModuleName.equals("rospy") || rosVars.contains(callModuleName)) {
						String callFuncName = ((NameTok) ((Attribute) call.func).attr).id;
						
						// Case for Subscriber generators
						if (Character.isUpperCase(callFuncName.charAt(0))) {
							for (FunctionDef funcDef: rosAPI) {
								if (((NameTok) funcDef.name).id.equals("__init__") &&
										funcDef.parent instanceof ClassDef && 
										((NameTok)((ClassDef) funcDef.parent).name).id.equals(callFuncName)) {
									// Assemble Call to Function Def
									Collection<stmtType> callCollection = callsApiMap.get(funcDef);
									if (callCollection == null) {
										callCollection = new LinkedList<stmtType>();
										callsApiMap.put(funcDef, callCollection);
									}
									callCollection.add(node);
								}
							}
						}
						// Case for publish method
						else {
							for (FunctionDef funcDef: rosAPI) {
								if (((NameTok) funcDef.name).id.equals(callFuncName)) {
									Collection<stmtType> callCollection = callsApiMap.get(funcDef);
									if (callCollection == null) {
										callCollection = new LinkedList<stmtType>();
										callsApiMap.put(funcDef, callCollection);
									}
									callCollection.add(node);
								}
							}
						}
					}
				}
			}
			
			return super.visitExpr(node);
		}
	}
	
	public Map<FunctionDef, Collection<stmtType>> getCalls(List<ParseOutput> parsedList, List<FunctionDef> rosAPI) {
		Map<FunctionDef, Collection<stmtType>> callsApiMap = new HashMap<>();
		CallVisitor visitor = new CallVisitor(rosAPI);
		
		for (ParseOutput output : parsedList) {
			Module ast = (Module) output.ast;
			
			try {
				ast.accept(visitor);
			} catch (Exception e) { e.printStackTrace(); }
		}
		return visitor.callsApiMap;
	}
	
	
	private static IDocument createIDocument(IFile ifile) throws CoreException {
		// IDocumentProvider is needed to translate IResource to IDocument. 
		// We need IDocument as an input to the PyParser.createCythonAst() function
		IDocumentProvider provider = new TextFileDocumentProvider();
		
		provider.connect(ifile);
		IDocument document = provider.getDocument(ifile);
		
		return document;
	}
	
	
	/*
	 * Pseudo path acquisition function
	 */
	private String getSitePackagesPath() {
		return "C:/Users/cometores/AppData/Local/Programs/Python/Python39/Lib/site-packages/rospy/";
	}
	
	private static void createLink(final java.io.File source, final IFile target, final IProgressMonitor monitor) throws CoreException {
		final IPath jarPath = new org.eclipse.core.runtime.Path(source.getAbsolutePath());
		target.createLink(jarPath, IResource.NONE, monitor);
	}
}
