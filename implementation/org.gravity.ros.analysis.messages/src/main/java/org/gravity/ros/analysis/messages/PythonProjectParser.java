package org.gravity.ros.analysis.messages;

import java.net.URI;
import java.net.URISyntaxException;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.eclipse.core.internal.resources.File;
import org.eclipse.core.resources.IFile;
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
import org.python.pydev.parser.jython.ast.FunctionDef;
import org.python.pydev.parser.jython.ast.Module;
import org.python.pydev.parser.jython.ast.NameTok;
import org.python.pydev.parser.jython.ast.VisitorBase;
import org.python.pydev.plugin.nature.PythonNature;
import org.python.pydev.shared_core.model.ISimpleNode;
import org.python.pydev.shared_core.parsing.BaseParser.ParseOutput;

public class PythonProjectParser {

	public List<ParseOutput> parse(IProject project) throws CoreException{
		
		// Get the PythonNature for the IProject we are going to parse
		IPythonPathNature iPythonPathNature = PythonNature.getPythonPathNature(project);
		IPythonNature iPythonNature = iPythonPathNature.getNature();

		// IDocumentProvider is needed to translate IResource to IDocument. 
		// We need IDocument as an input to the PyParser.createCythonAst() function
		IDocumentProvider provider = new TextFileDocumentProvider();
		
		LinkedList<ParseOutput> parsedList = new LinkedList<>();
		
		Set<IResource> iResourceList = (Set<IResource>) iPythonPathNature.getProjectSourcePathFolderSet();

		
		/* Способ не работает, добавляет путь к проекту*/
//		IPath path = new Path("C:/Users/cometores/AppData/Local/Programs/Python/Python39/Lib/site-packages/rospy");
//		IResource resource = project.getFolder(path);
//		iResourceList.add(resource);
		
		
		
		// 11111111111111111
//		java.io.File file = new java.io.File("C:/Users/cometores/AppData/Local/Programs/Python/Python39/Lib/site-packages/rospy/client.py");
//		IFile iFile = project.getFolder("libs").getFile("filename");
//		
//		createLink(file, iFile, null);
		// 11111111111111111111111111
		
		/* 
		 * for function name in JSON:
		 *  mache: IFile
		 *   IFile -> IResource
		 *   Set<IResource>.add(
		 */
		
		
		// for pfad i:
		//	  IFile iFile = project.getFile("libs/filename");
		//    parseCython(IFile)
		//       suche nach Function def mit name
		
		
		
		/* Тоже не работает */
//		IWorkspaceRoot root = project.getWorkspace().getRoot(); 
//		IResource rospy = root.findMember("/libs/rospy");
		
		
		// for (IResource scr : iPythonPathNature.getProjectSourcePathFolderSet()) {
		
		
		
		
		for (IResource scr : iResourceList) {
			scr.accept(new IResourceVisitor() {
				@Override
				public boolean visit(IResource resource) throws CoreException {
					if (resource instanceof IFile) {
						IFile ifile = (IFile) resource;
						String ifileName = ifile.getName(); // TEST LINE

						// Ignoriere __init__.py files
						if (ifile.getFileExtension().equals("py") && !ifileName.equals("__init__.py")) {
							System.out.println(ifileName); // TEST LINE
							
							provider.connect(ifile);
							IDocument document = provider.getDocument(ifile);

							try {
								ParseOutput parseOutput = PyParser
										.createCythonAst(new ParserInfo(document, iPythonNature));
								parsedList.add(parseOutput);
							} catch (MisconfigurationException e) {
								// TODO Auto-generated catch block
								e.printStackTrace();
							}
						}
					}
					return true;
				}
			});
		}
		return parsedList;
	}
	
	class MyVisitor extends Visitor {
		public List<FunctionDef> api = new LinkedList<>();
		List<String> apiNeeded;
		String className = null;
		boolean inClass = false;
		
		public MyVisitor(String _className, List<String> _apiNeeded) {
			apiNeeded = _apiNeeded;
			className = _className;
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
		
		/* Special visit for FunctionDef only. In the other case, we bypass it as usual. */
		@Override
		public Object visitFunctionDef(FunctionDef node) throws Exception {
			//
			if (inClass && apiNeeded.contains( ((NameTok) node.name).id )) {              
				api.add(node);
			}

			// TODO: Filter if is searched call
			return super.visitFunctionDef(node);
		}
	}
	
	public List<FunctionDef> getRosAPI(List<ParseOutput> asts) {
		
		
		// Collection of all Function Definitions names we need
		List<String> apiNeeded = getNeededAPIfromJSON();
		
		MyVisitor visitor = new MyVisitor("RosNode", apiNeeded);
		// Go through AST, stopping at FuncDef. If we find the needed function, add it to the collection.
		for (ParseOutput output : asts) {
			Module ast = (Module) output.ast;
			
			try {
				ast.accept(visitor);
			} catch (Exception e) { e.printStackTrace(); }
		}
		
		return visitor.api;
	}
	
	
	public Map<FunctionDef, Collection<Call>> getCalls(List<ParseOutput> parsedList, List<FunctionDef> def) {
		for (ParseOutput output : parsedList) {
			Module ast = (Module) output.ast;
			try {
				ast.accept(new Visitor() {

					@Override
					public Object visitCall(Call node) throws Exception {
						// TODO: Filter if is searched call
						if(def.contains(node)) {
							// Store node + call in map
						}
						return super.visitCall(node);
					}

				});
			} catch (Exception e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
		return null;
	}
	
	private List<String> getNeededAPIfromJSON() {
		List<String> apiNeeded = new LinkedList<>();
		apiNeeded.add("listener");
		apiNeeded.add("callback");
		apiNeeded.add("talker");
		return apiNeeded;
	}
	
	public static void createLink(final java.io.File source, final IFile target, final IProgressMonitor monitor) throws CoreException {
		final IPath jarPath = new org.eclipse.core.runtime.Path(source.getAbsolutePath());
		target.createLink(jarPath, IResource.FILE, monitor);
	}
	
	
	/* Тоже какая-то хуйня */
//	private void addResources(Set<IResource> iResourceList) {
//		IWorkspace ws = ResourcesPlugin.getWorkspace();
//		IProject project = ws.getRoot().getProject("External Files");
//		if (!project.exists())
//		    project.create(null);
//		if (!project.isOpen())
//		    project.open(null);
//		Shell shell = window.getShell();
//		String name = new FileDialog(shell, SWT.OPEN).open();
//		if (name == null)
//		    return;
//		IPath location = new Path(name);
//		IFile file = project.getFile(location.lastSegment());
//		file.createLink(location, IResource.NONE, null);
//		IWorkbenchPage page = window.getActivePage();
//		if (page != null)
//		    page.openEditor(file);
//	}
}
