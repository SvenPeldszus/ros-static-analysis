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
import org.gravity.ros.analysis.messages.Dataclasses.AstModuleInfo;
import org.gravity.ros.analysis.messages.Dataclasses.FunctionInfo;
import org.gravity.ros.analysis.messages.Dataclasses.TopicInfo;
import org.gravity.ros.analysis.messages.Visitors.AliasingVisitor;
import org.gravity.ros.analysis.messages.Visitors.CallVisitor;
import org.gravity.ros.analysis.messages.Visitors.FunctionVisitor;
import org.gravity.ros.analysis.messages.Visitors.ROSApiVisitor;
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
	private Dataclasses dataclasses= new Dataclasses();
	private Visitors visitors = new Visitors();
	
	
	/*
	 * Function to get the AST of all the files of the project you want to parse
	 * @param  project   The project we will be parsing
	 */
	public List<AstModuleInfo> parse(IProject project) throws CoreException{
		
		// Get the PythonNature for the IProject we are going to parse. After collect all files and folders of project
		IPythonPathNature iPythonPathNature = PythonNature.getPythonPathNature(project);
		IPythonNature iPythonNature = iPythonPathNature.getNature();
		Set<IResource> iResourceList = (Set<IResource>) iPythonPathNature.getProjectSourcePathFolderSet();

		// IDocumentProvider is needed to translate IResource to IDocument. 
		// We need IDocument as an input to the PyParser.createCythonAst() function
		IDocumentProvider provider = new TextFileDocumentProvider();
		
		// Collection of all ast's of files in project we need to return
		LinkedList<AstModuleInfo> parsedList = new LinkedList<>();
		
		
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
							String moduleName = ifile.getName();
							System.out.println(moduleName); // File name output for debugging
							IDocument document = createIDocument(ifile); // We need an IDocument for the parser

							// Parsing the needed file with PyParser
							try {
								String folderName = ifile.getParent().getName();
								ParseOutput parseOutput = PyParser.reparseDocument(new ParserInfo(document, iPythonNature));
								parsedList.add(dataclasses.new AstModuleInfo(parseOutput, moduleName, folderName));
							} catch (MisconfigurationException e) { e.printStackTrace(); }
						}
					}
					return true;
				}
			});
		}
		return parsedList;
	}
	
	
	/*
	 * Get the list of ROS API functions described in JSON.
	 * @param  project   The project we will be parsing
	 */
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
		    
		    ROSApiVisitor visitor = visitors.new ROSApiVisitor(className, funcName);
		    ast.accept(visitor);
		    
		    api.add(visitor.api);
		}
		
		return api;
	}

	
	/*
	 * Function to find all ROS Api calls in a project and collect them into {Topic - ROS Nodes} structure
	 */
	public Map<String, TopicInfo> getCalls(List<AstModuleInfo> moduleInfo, List<FunctionDef> rosAPI) {
		
		CallVisitor visitor = visitors.new CallVisitor(rosAPI);
		
		// Find functions in all modules
		Map<String, FunctionInfo> functions = getFunctions(moduleInfo);
		visitor.globalFunctions = functions;
		
		getAliasing(moduleInfo);
		
		for (AstModuleInfo module : moduleInfo) {
			Module ast = (Module) module.ast.ast;
			
			try {
				visitor.moduleInfo = module;
				ast.accept(visitor);
			} catch (Exception e) { e.printStackTrace(); }
		}
		
		return visitor.topicsMap;
	}
	
	
	private Map<String, FunctionInfo> getFunctions(List<AstModuleInfo> moduleInfo) {
		FunctionVisitor visitor = visitors.new FunctionVisitor();

		for (AstModuleInfo module : moduleInfo) {
			Module ast = (Module) module.ast.ast;
			
			try {
				visitor.moduleInfo = module; 
				ast.accept(visitor);
			} catch (Exception e) { e.printStackTrace(); }
		}
		return visitor.globalFunctions;
	}
	
	private void getAliasing(List<AstModuleInfo> moduleInfo) {
		AliasingVisitor visitor = visitors.new AliasingVisitor();

		for (AstModuleInfo module : moduleInfo) {
			Module ast = (Module) module.ast.ast;
			
			try {
				visitor.moduleInfo = module; 
				ast.accept(visitor);
			} catch (Exception e) { e.printStackTrace(); }
		}
	}
	
	
	/*
	 * Supporting function for creating a document from a file
	 */
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
	
	
	/*
	 * Support function for creating a link to a file from the Python library
	 */
	private static void createLink(final java.io.File source, final IFile target, final IProgressMonitor monitor) throws CoreException {
		final IPath jarPath = new org.eclipse.core.runtime.Path(source.getAbsolutePath());
		target.createLink(jarPath, IResource.NONE, monitor);
	}
}
