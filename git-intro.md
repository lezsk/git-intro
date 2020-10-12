git的应用（windows系统）

安装git，在某个目录下右键打开git bash终端。

## 术语表

-  **git**: 一个开源的分布式版本控制系统
-  **GitHub**: 一个托管和协作管理 Git 仓库的平台
-  **commit 提交**: 一个 Git 对象，是你整个仓库的快照的哈希值
-  **branch 分支**: 一个轻型可移动的 commit 指针
-  **clone**: 一个仓库的本地版本，包含所有提交和分支
-  **remote 远端**: 一个 GitHub 上的公共仓库，所有小组成员通过它来交换修改
- **HEAD**: 代表你当前的工作目录。使用`git checkout` 可移动 HEAD 指针到不同的分支、标记(tags)或提交

一、建立本地仓库和提交

在gitee、github或者其他托管网站上复制目标仓库的地址

以下都会以git-intro为例说明

在终端中输入 

```
`git clone https://github.com/lezsk/git-intro.git`
 cd git-intro
```

就进入了仓库,看一下目前有的目录

```
$ ls
README.md  source/
```

假设在git-intro目录下添加一个git-intro.md文件，得到

```
$ ls
git-intro.md  README.md  source/
```

用`git status`查看更改信息，系统会自动列出推荐的操作

```
$ git status
On branch main
Your branch is up to date with 'origin/main'./*这是我们的远程分支目录*/

Untracked files:
  (use "git add <file>..." to include in what will be committed)
        git-intro.md

nothing added to commit but untracked files present (use "git add" to track)

```

可以使用`git add git-intro.md`或者`git add .`或者`git add -A`来添加本目录下untracked file

```
$ git add .
$ git status
On branch main
Your branch is up to date with 'origin/main'.

Changes to be committed:
  (use "git restore --staged <file>..." to unstage)
        new file:   git-intro.md
```

此时我们的本地仓库中的/.../git-intro/git-intro.md从unstaged变成staged状态（可以在gitkraken右边栏看到此状态）

然后用git commit命令进行本地提交，附加更新说明文字。

```
$ git commit -m "add git-intro.md"
[main 4cd223b] add git-intro.md
 1 file changed, 29 insertions(+)
 create mode 100644 git-intro.md

```

最后git push命令同步到github

此时，同事在远端做了更改，我在本地做了不同的更改，如图

![](C:\Users\zz\Downloads\different commit.JPG)

用以下命令查看不同

```
$ git fetch 
$ git log -p HEAD..FETCH_HEAD
```

git pull 相当于git fetch和git merge结合

git pull如果有冲突，会提示更改。此时利用以下命令。可以之后手动修改相关文件

```
$ git diff
diff --cc source/readme.txt
index 139cc97,bad4b32..0000000
--- a/source/readme.txt
+++ b/source/readme.txt
@@@ -1,5 -1,5 +1,9 @@@
  /*!
++<<<<<<< HEAD
 +    \we add this linee
++=======
+     \we add this line
++>>>>>>> 3dc93dfbe49f8ab8c41e14e867f7cc5b523f2b69
      \file  readme.txt
      \brief description of the ADC regular channel with DMA

```

修改完成后再commit以下就可以完成之前pull的merge操作



PS: 关于diff命令的一点说明：



现在要给代码添加一个新功能，我新建一个dev分支

## 分支

分支是使用 Git 工作的一个重要部分。你做的任何提交都会发生在当前“checked out”到的分支上。使用 `git status` 查看那是哪个分支。

```
$ git branch dev
```

创建一个新分支

```
$ git checkout dev
```

切换到指定分支并更新工作目录(working directory)

```
$ git merge [branch]
```

将指定分支的历史合并到当前分支。这通常在拉取请求(PR)中完成，但也是一个重要的 Git 操作。

```
$ git branch -d dev
```

删除指定分支